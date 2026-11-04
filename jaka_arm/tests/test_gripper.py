# SPDX-License-Identifier: Apache-2.0
# -*- coding: utf-8 -*-
"""Unit tests for the JakaGripper extended-IO backend against a FakeRC stub.

The controller's RS485 signal table is unusable on firmware 3.3.15 (legacy
get_rs485_signal_info segfaults, grpc returns eight empty slots, update returns
260), so the gripper is driven through the Coboπ Modbus-slave channels:

    read  get_analog_input(IO_EXTEND, ai_idx, submodule_id)
    write set_analog_output(IO_EXTEND, ao_idx, value, submodule_id)

These tests pin the channel mapping, the write order, error propagation and the
closure-based position semantics (1000 = closed, 0 = open).
"""
import contextlib
import io
import unittest
from unittest.mock import patch

from jaka_arm.gripper import EXTIO_AI, EXTIO_AO, IO_EXTEND, REG, JakaGripper

SUB = 54052


class FakeRC:
    """Minimal jkrc.RC stand-in for the extended-IO calls."""

    def __init__(self, ai=None, running=True, read_code=0, write_code=0):
        self.ai = dict(ai or {})   # channel index -> value
        self.running = running
        self.read_code = read_code
        self.write_code = write_code
        self.reads = []            # (type, index, submodule_id)
        self.writes = []           # (type, index, value, submodule_id)
        self.power_on_calls = 0

    def power_on(self):
        self.power_on_calls += 1
        return (0,)

    def is_extio_running(self):
        return (0, 1 if self.running else 0)

    def get_analog_input(self, iotype, index, submodule_id):
        self.reads.append((iotype, index, submodule_id))
        if self.read_code:
            return (self.read_code,)
        return (0, float(self.ai.get(index, 0)))

    def set_analog_output(self, iotype, index, value, submodule_id):
        self.writes.append((iotype, index, value, submodule_id))
        return (self.write_code,)


class TestGripperReads(unittest.TestCase):
    def test_reads_use_mapped_ai_channels(self):
        rc = FakeRC(ai={0: 1002, 1: 40, 2: 1})
        g = JakaGripper(robot=rc)
        self.assertEqual(g.read_position(), 1002)
        self.assertEqual(g.read_current(), 40)
        self.assertEqual(g.read_state(), 1)
        self.assertEqual(
            rc.reads,
            [(IO_EXTEND, EXTIO_AI["position"], SUB),
             (IO_EXTEND, EXTIO_AI["current"], SUB),
             (IO_EXTEND, EXTIO_AI["state"], SUB)],
        )

    def test_unmapped_registers_return_none(self):
        rc = FakeRC(ai={0: 500})
        g = JakaGripper(robot=rc)
        self.assertIsNone(g.read_force_fx())
        self.assertIsNone(g.read_open_position())
        self.assertIsNone(g.read_close_position())
        self.assertEqual(rc.reads, [])  # no bus traffic for unknown channels

    def test_read_error_returns_none(self):
        g = JakaGripper(robot=FakeRC(read_code=10003))
        self.assertIsNone(g.read_position())

    def test_opening_follows_closure_semantics(self):
        g = JakaGripper(robot=FakeRC(ai={0: 1000}))
        self.assertAlmostEqual(g.opening_m(), 0.0, places=6)
        g = JakaGripper(robot=FakeRC(ai={0: 0}))
        self.assertAlmostEqual(g.opening_m(), 0.098, places=6)
        g = JakaGripper(robot=FakeRC(ai={0: 500}))
        self.assertAlmostEqual(g.opening_m(), 0.049, places=6)


class TestGripperWrites(unittest.TestCase):
    def test_grip_writes_speed_then_action(self):
        rc = FakeRC()
        g = JakaGripper(robot=rc)
        self.assertEqual(g.grip(1100), 0)
        self.assertEqual(
            rc.writes,
            [(IO_EXTEND, EXTIO_AO["adaptive_speed"], 1100.0, SUB),
             (IO_EXTEND, EXTIO_AO["action"], 1.0, SUB)],
        )

    def test_release_writes_action_zero(self):
        rc = FakeRC()
        g = JakaGripper(robot=rc)
        self.assertEqual(g.release(900), 0)
        self.assertEqual(rc.writes[-1], (IO_EXTEND, EXTIO_AO["action"], 0.0, SUB))

    def test_speed_failure_skips_action(self):
        rc = FakeRC(write_code=10007)
        g = JakaGripper(robot=rc)
        self.assertEqual(g.grip(1000), 10007)
        self.assertEqual(len(rc.writes), 1)  # action never sent

    def test_action_failure_is_propagated(self):
        class HalfFail(FakeRC):
            def set_analog_output(self, iotype, index, value, submodule_id):
                super().set_analog_output(iotype, index, value, submodule_id)
                return (0,) if index == EXTIO_AO["adaptive_speed"] else (10007,)

        g = JakaGripper(robot=HalfFail())
        self.assertEqual(g.grip(1000), 10007)

    def test_unsupported_setters_return_minus_one(self):
        g = JakaGripper(robot=FakeRC())
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            self.assertEqual(g.set_target_force(50), -1)
            self.assertEqual(g.set_control_mode(1), -1)
            self.assertEqual(g.calibrate_fingertip(), -1)
        self.assertEqual(buf.getvalue().count("set_target_force"), 1)  # warn once


class TestGripperLifecycle(unittest.TestCase):
    def test_submodule_id_is_passed_through(self):
        rc = FakeRC(ai={0: 10})
        g = JakaGripper(robot=rc, submodule_id=12345)
        g.read_position()
        g.grip(1000)
        self.assertTrue(all(call[2] == 12345 for call in rc.reads))
        self.assertTrue(all(call[3] == 12345 for call in rc.writes))

    def test_power_on_powers_and_waits_for_current(self):
        rc = FakeRC(ai={EXTIO_AI["current"]: 42})
        with patch("jaka_arm.gripper.time.sleep"):
            JakaGripper(robot=rc).power_on()
        self.assertEqual(rc.power_on_calls, 1)
        self.assertEqual(len(rc.reads), 1)  # first read had current, loop exits

    def test_power_on_warns_when_extio_not_running(self):
        rc = FakeRC(ai={}, running=False)
        buf = io.StringIO()
        with patch("jaka_arm.gripper.time.sleep"), contextlib.redirect_stdout(buf):
            JakaGripper(robot=rc).power_on()
        self.assertIn("扩展 IO 从站未运行", buf.getvalue())
        self.assertEqual(len(rc.reads), 8)  # readiness loop exhausted

    def test_close_does_not_logout_in_shared_mode(self):
        class Shared(FakeRC):
            def __init__(self):
                super().__init__()
                self.logged_out = False

            def logout(self):
                self.logged_out = True
                return (0,)

        rc = Shared()
        JakaGripper(robot=rc).close()
        self.assertFalse(rc.logged_out)

    def test_reg_map_is_untouched(self):
        self.assertEqual(REG["position"], 0x0008)
        self.assertEqual(REG["action"], 0x000B)


if __name__ == "__main__":
    unittest.main()

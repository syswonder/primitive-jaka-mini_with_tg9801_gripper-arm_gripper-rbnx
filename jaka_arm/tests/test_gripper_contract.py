# SPDX-License-Identifier: Apache-2.0
# -*- coding: utf-8 -*-
"""Unit tests for the standard gripper contract mapping (no hardware)."""
import math
import unittest

from jaka_arm import gripper_contract as gc


class TestRelativeSpeed(unittest.TestCase):
    def test_zero_selects_configured_default(self):
        self.assertEqual(gc.relative_speed(0.0, 1100), 1100)

    def test_range_endpoints(self):
        self.assertEqual(gc.relative_speed(1.0, 1100), gc.SPEED_MAX)
        self.assertEqual(gc.relative_speed(0.5, 1100), (gc.SPEED_MIN + gc.SPEED_MAX) // 2)

    def test_monotonic(self):
        speeds = [gc.relative_speed(v / 10, 1100) for v in range(1, 11)]
        self.assertEqual(speeds, sorted(speeds))
        self.assertEqual(len(set(speeds)), len(speeds))


class TestValidateGrip(unittest.TestCase):
    def test_adaptive_form_accepted(self):
        self.assertIsNone(gc.validate_grip(0.0, 0.0, False, 0.0))
        self.assertIsNone(gc.validate_grip(0.0, 1.0, False, 0.0))

    def test_effort_rejected_when_not_configurable(self):
        self.assertFalse(gc.GRIP_EFFORT_CONFIGURABLE)
        reason = gc.validate_grip(0.5, 0.0, False, 0.0)
        self.assertIn("relative_effort", reason)

    def test_expected_width_rejected(self):
        reason = gc.validate_grip(0.0, 0.0, True, 0.05)
        self.assertIn("expected object width", reason)

    def test_relative_values_must_be_finite_and_bounded(self):
        for bad in (math.nan, math.inf, -0.1, 1.5):
            self.assertIsNotNone(gc.validate_grip(0.0, bad, False, 0.0))
            self.assertIsNotNone(gc.validate_grip(bad, 0.0, False, 0.0))


class TestValidateOpen(unittest.TestCase):
    def test_accepts_unit_interval(self):
        self.assertIsNone(gc.validate_open(0.0))
        self.assertIsNone(gc.validate_open(1.0))

    def test_rejects_out_of_range(self):
        self.assertIn("[0, 1]", gc.validate_open(2.0))
        self.assertIsNotNone(gc.validate_open(math.nan))


class TestDeviceStateMapping(unittest.TestCase):
    """Register 0x0006: 0 idle / 1 moving / 2 holding / 3 slipping."""

    def test_grasp_state(self):
        self.assertEqual(gc.grasp_state(0), gc.GRASP_NOT_HOLDING)
        self.assertEqual(gc.grasp_state(1), gc.GRASP_NOT_HOLDING)
        self.assertEqual(gc.grasp_state(2), gc.GRASP_HOLDING)
        self.assertEqual(gc.grasp_state(3), gc.GRASP_SLIPPING)
        self.assertEqual(gc.grasp_state(None), gc.GRASP_UNKNOWN)
        self.assertEqual(gc.grasp_state(9), gc.GRASP_UNKNOWN)

    def test_motion_state(self):
        self.assertEqual(gc.motion_state(1), gc.MOTION_MOVING)
        for stopped in (0, 2, 3):
            self.assertEqual(gc.motion_state(stopped), gc.MOTION_STOPPED)
        self.assertEqual(gc.motion_state(None), gc.MOTION_UNKNOWN)
        self.assertEqual(gc.motion_state(9), gc.MOTION_UNKNOWN)

    def test_moving_never_counts_as_contact(self):
        # 1 means the jaws are travelling: no established grip, and a
        # successful dispatch alone must not be mistaken for contact.
        self.assertNotEqual(gc.grasp_state(1), gc.GRASP_HOLDING)


class TestMetadata(unittest.TestCase):
    def test_only_declarable_facts(self):
        self.assertEqual(gc.MIN_OPENING_M, 0.0)
        self.assertEqual(gc.MAX_OPENING_M, 0.098)
        self.assertTrue(gc.OPENING_WIDTH_RANGE_VALID)
        self.assertFalse(gc.GRIPPING_FORCE_RANGE_VALID)
        self.assertFalse(gc.GRIP_WITH_EXPECTED_WIDTH_SUPPORTED)
        self.assertTrue(gc.GRIP_WITHOUT_EXPECTED_WIDTH_SUPPORTED)
        self.assertTrue(gc.GRIP_SPEED_CONFIGURABLE)
        self.assertTrue(gc.OPEN_SPEED_CONFIGURABLE)
        self.assertFalse(gc.MOVE_SPEED_CONFIGURABLE)


if __name__ == "__main__":
    unittest.main()

# SPDX-License-Identifier: Apache-2.0
# -*- coding: utf-8 -*-
"""Unit tests for the standard-gripper-contract mapping.

The rules here decide what `gripper/info` advertises and what `gripper/grip`
rejects, so they are pinned independently of rclpy and the generated IDL.
"""
import unittest

from jaka_arm import gripper_contract as gc


class TestRelativeSpeed(unittest.TestCase):
    def test_zero_selects_default(self):
        self.assertEqual(gc.relative_speed(0.0, 1100), 1100)

    def test_full_speed_hits_max(self):
        self.assertEqual(gc.relative_speed(1.0, 1100), gc.SPEED_MAX)

    def test_midpoint_is_between_bounds(self):
        speed = gc.relative_speed(0.5, 1100)
        self.assertGreater(speed, gc.SPEED_MIN)
        self.assertLess(speed, gc.SPEED_MAX)


class TestGripValidation(unittest.TestCase):
    def test_adaptive_form_is_accepted(self):
        self.assertIsNone(gc.validate_grip(0.0, 0.0, False, 0.0))
        self.assertIsNone(gc.validate_grip(0.0, 0.7, False, 0.0))

    def test_out_of_range_values_are_rejected(self):
        self.assertIsNotNone(gc.validate_grip(0.0, 1.5, False, 0.0))
        self.assertIsNotNone(gc.validate_grip(0.0, -0.1, False, 0.0))
        self.assertIsNotNone(gc.validate_grip(float("nan"), 0.5, False, 0.0))

    def test_relative_effort_is_rejected_when_not_configurable(self):
        reason = gc.validate_grip(0.4, 0.0, False, 0.0)
        self.assertIsNotNone(reason)
        self.assertIn("relative_effort", reason)

    def test_expected_width_is_rejected_when_unsupported(self):
        reason = gc.validate_grip(0.0, 0.0, True, 0.04)
        self.assertIsNotNone(reason)
        self.assertIn("expected object width", reason)

    def test_open_validation(self):
        self.assertIsNone(gc.validate_open(0.0))
        self.assertIsNone(gc.validate_open(0.25))
        self.assertIsNotNone(gc.validate_open(2.0))


class TestStateMapping(unittest.TestCase):
    def test_grasp_state_follows_device_register(self):
        self.assertEqual(gc.grasp_state(None), gc.GRASP_UNKNOWN)
        self.assertEqual(gc.grasp_state(0), gc.GRASP_NOT_HOLDING)
        self.assertEqual(gc.grasp_state(1), gc.GRASP_HOLDING)

    def test_motion_state_is_unknown_without_two_readings(self):
        self.assertEqual(gc.motion_state(None, 0.05), gc.MOTION_UNKNOWN)
        self.assertEqual(gc.motion_state(0.05, None), gc.MOTION_UNKNOWN)

    def test_motion_state_detects_travel(self):
        self.assertEqual(gc.motion_state(0.05, 0.05), gc.MOTION_STOPPED)
        self.assertEqual(gc.motion_state(0.0, 0.098), gc.MOTION_MOVING)

    def test_motion_epsilon_ignores_sensor_jitter(self):
        jitter = gc.MOTION_EPS_M / 2
        self.assertEqual(gc.motion_state(0.05, 0.05 + jitter), gc.MOTION_STOPPED)


class TestInfoMetadata(unittest.TestCase):
    def test_advertises_only_what_the_device_can_do(self):
        self.assertFalse(gc.GRIP_EFFORT_CONFIGURABLE)
        self.assertFalse(gc.MOVE_SPEED_CONFIGURABLE)
        self.assertTrue(gc.GRIP_SPEED_CONFIGURABLE)
        self.assertTrue(gc.OPEN_SPEED_CONFIGURABLE)
        self.assertFalse(gc.GRIPPING_FORCE_RANGE_VALID)
        self.assertFalse(gc.GRIP_WITH_EXPECTED_WIDTH_SUPPORTED)
        self.assertTrue(gc.GRIP_WITHOUT_EXPECTED_WIDTH_SUPPORTED)

    def test_width_range_matches_the_stroke(self):
        self.assertTrue(gc.OPENING_WIDTH_RANGE_VALID)
        self.assertEqual(gc.MIN_OPENING_M, 0.0)
        self.assertAlmostEqual(gc.MAX_OPENING_M, 0.098, places=6)


if __name__ == "__main__":
    unittest.main()

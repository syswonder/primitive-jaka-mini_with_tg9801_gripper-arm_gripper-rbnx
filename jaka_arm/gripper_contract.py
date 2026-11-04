# SPDX-License-Identifier: Apache-2.0
# -*- coding: utf-8 -*-
"""Mapping between the standard ``robonix/primitive/gripper/*`` contracts and
the TG-9801 gripper on the Coboπ extended-IO slave.

Deliberately free of ROS imports: every rule here is a pure function so it can
be unit-tested without rclpy or the generated IDL packages.

What the device can and cannot do drives the declared capabilities:

* adaptive mode exposes a speed setting and a binary grip/release action; the
  closing force is whatever the device applies, and the datasheet gives no
  per-face force in newtons, so ``relative_effort`` is rejected rather than
  mapped to a made-up number;
* there is no absolute opening-width command, so ``move`` is not declared;
* there is no halt command, so ``stop`` is not declared;
* a grip with an expected object width would need width-targeted motion, so
  only the adaptive form is accepted.
"""
from __future__ import annotations

import math

# ---- GetGripperInfo metadata ------------------------------------------------
MIN_OPENING_M = 0.0
MAX_OPENING_M = 0.098          # TG-9801 stroke, matches GRIPPER_STROKE_M

GRIP_EFFORT_CONFIGURABLE = False
GRIP_SPEED_CONFIGURABLE = True
OPEN_SPEED_CONFIGURABLE = True
MOVE_SPEED_CONFIGURABLE = False
OPENING_WIDTH_RANGE_VALID = True
GRIPPING_FORCE_RANGE_VALID = False
GRIP_WITH_EXPECTED_WIDTH_SUPPORTED = False
GRIP_WITHOUT_EXPECTED_WIDTH_SUPPORTED = True

# ---- device speed range -----------------------------------------------------
SPEED_MIN = 200
SPEED_MAX = 1500

# ---- GripperState enums -----------------------------------------------------
GRASP_UNKNOWN = 0
GRASP_NOT_HOLDING = 1
GRASP_HOLDING = 2
GRASP_SLIPPING = 3

MOTION_UNKNOWN = 0
MOTION_MOVING = 1
MOTION_STOPPED = 2

# Jaw travel between two acquisitions above this counts as motion, m.
MOTION_EPS_M = 0.0005


def relative_speed(relative: float, default_speed: int) -> int:
    """Map a ``(0, 1]`` relative speed to the device speed range.

    Zero selects the configured default (the contract forbids reusing a value
    from an earlier command).
    """
    if relative <= 0:
        return int(default_speed)
    span = SPEED_MAX - SPEED_MIN
    return int(round(SPEED_MIN + span * float(relative)))


def _check_relative(name: str, value: float) -> str | None:
    if not math.isfinite(value):
        return f"{name} must be finite"
    if value < 0.0 or value > 1.0:
        return f"{name} must be in [0, 1]"
    return None


def validate_grip(relative_effort: float, relative_speed: float,
                  expected_object_width_valid: bool,
                  expected_object_width_m: float) -> str | None:
    """Return the rejection reason, or None when the request may be dispatched."""
    err = _check_relative("relative_effort", relative_effort)
    if err:
        return err
    err = _check_relative("relative_speed", relative_speed)
    if err:
        return err
    if relative_effort > 0.0 and not GRIP_EFFORT_CONFIGURABLE:
        return "relative_effort is not configurable on this gripper"
    if expected_object_width_valid:
        if not GRIP_WITH_EXPECTED_WIDTH_SUPPORTED:
            return "grip with an expected object width is not supported"
        if not math.isfinite(expected_object_width_m) or expected_object_width_m < 0:
            return "expected_object_width_m must be finite and nonnegative"
    return None


def validate_open(relative_speed: float) -> str | None:
    return _check_relative("relative_speed", relative_speed)


def grasp_state(device_state: int | None) -> int:
    """Map the device's holding-state register to ``GraspState``.

    The TG-9801 reports 0 (released / no object) and 1 (holding). It never
    reports slipping, so that value is never produced here.
    """
    if device_state is None:
        return GRASP_UNKNOWN
    return GRASP_HOLDING if int(device_state) else GRASP_NOT_HOLDING


def motion_state(previous_m, current_m) -> int:
    """Infer jaw motion from two consecutive acquired openings.

    The contract wants a current observation, not a command result: equal
    widths 0.2 s apart mean no observed travel, not arrival.
    """
    if previous_m is None or current_m is None:
        return MOTION_UNKNOWN
    if abs(float(current_m) - float(previous_m)) > MOTION_EPS_M:
        return MOTION_MOVING
    return MOTION_STOPPED

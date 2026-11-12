# SPDX-License-Identifier: Apache-2.0
# -*- coding: utf-8 -*-
"""Mapping between the standard ``robonix/primitive/gripper/*`` contracts and
the TG-9801 adaptive gripper on the JAKA controller's end RS485 channel.

Deliberately free of ROS and jkrc imports: every rule here is a pure function
so it can be unit-tested without hardware.

What the device can and cannot do drives the declared capabilities:

* adaptive mode is binary (grip / release) with a speed setting; the closing
  force is a controller-wide setting applied once at connect, and the
  datasheet gives no per-face force in newtons, so ``relative_effort`` is
  rejected rather than mapped to a made-up number;
* there is no verified absolute opening-width command, so ``move`` is not
  declared;
* there is no halt command, so ``stop`` is not declared;
* a grip with an expected object width would need width-targeted motion, so
  only the adaptive form is accepted.

The holding-state register (0x0006) reports 0 idle / 1 moving / 2 holding /
3 slipping, which is exactly the observation GraspState and MotionState ask
for: both are read from the device, never inferred from jaw travel or from
the result of a command.
"""
from __future__ import annotations

import math

# ---- GetGripperInfo metadata ------------------------------------------------
MIN_OPENING_M = 0.0
MAX_OPENING_M = 0.098          # TG-9801 stroke (two-finger gap), matches
                               # driver.GRIPPER_STROKE_M

GRIP_EFFORT_CONFIGURABLE = False
GRIP_SPEED_CONFIGURABLE = True
OPEN_SPEED_CONFIGURABLE = True
MOVE_SPEED_CONFIGURABLE = False
OPENING_WIDTH_RANGE_VALID = True
GRIPPING_FORCE_RANGE_VALID = False
GRIP_WITH_EXPECTED_WIDTH_SUPPORTED = False
GRIP_WITHOUT_EXPECTED_WIDTH_SUPPORTED = True

# ---- device speed range (TG-9801 自适应/力位 speed registers) ----------------
SPEED_MIN = 200
SPEED_MAX = 1500

# ---- device holding-state register 0x0006 -----------------------------------
DEVICE_IDLE = 0
DEVICE_MOVING = 1
DEVICE_HOLDING = 2
DEVICE_SLIPPING = 3

# ---- GripperState enums -----------------------------------------------------
GRASP_UNKNOWN = 0
GRASP_NOT_HOLDING = 1
GRASP_HOLDING = 2
GRASP_SLIPPING = 3

MOTION_UNKNOWN = 0
MOTION_MOVING = 1
MOTION_STOPPED = 2


def relative_speed(relative: float, default_speed: int) -> int:
    """Map a ``(0, 1]`` relative speed onto the device speed range.

    Zero selects the configured default, never a value left by an earlier
    command (the contract forbids carrying state between commands).
    """
    if relative <= 0:
        return int(default_speed)
    return int(round(SPEED_MIN + (SPEED_MAX - SPEED_MIN) * float(relative)))


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
        if (not math.isfinite(expected_object_width_m)
                or expected_object_width_m < 0.0):
            return "expected_object_width_m must be finite and nonnegative"
    return None


def validate_open(relative_speed: float) -> str | None:
    return _check_relative("relative_speed", relative_speed)


def grasp_state(device_state: int | None) -> int:
    """Map the holding-state register onto ``GraspState``.

    2 (夹稳) is the only value that establishes contact; 3 (打滑) is a contact
    that is slipping; 0 (空闲) and 1 (运动中) confirm no established grip.
    Anything else - including a failed read - is UNKNOWN, never guessed.
    """
    if device_state is None:
        return GRASP_UNKNOWN
    state = int(device_state)
    if state == DEVICE_HOLDING:
        return GRASP_HOLDING
    if state == DEVICE_SLIPPING:
        return GRASP_SLIPPING
    if state in (DEVICE_IDLE, DEVICE_MOVING):
        return GRASP_NOT_HOLDING
    return GRASP_UNKNOWN


def motion_state(device_state: int | None) -> int:
    """Map the same register onto ``MotionState``.

    The register's 1 (运动中) is the device's own jaw-motion report; every
    other known state means the jaws are not moving. STOPPED deliberately
    makes no claim about arrival, contact or a safety-rated stop.
    """
    if device_state is None:
        return MOTION_UNKNOWN
    state = int(device_state)
    if state == DEVICE_MOVING:
        return MOTION_MOVING
    if state in (DEVICE_IDLE, DEVICE_HOLDING, DEVICE_SLIPPING):
        return MOTION_STOPPED
    return MOTION_UNKNOWN

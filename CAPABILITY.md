# jaka_arm capability notes

JAKA 6-DOF arm (jkrc SDK over TCP) with a TG-9801 adaptive gripper on the
end RS485 channel. Provider ID is the deploy instance `name` (default
`jaka_arm`).

## Frames and units

- **Base frame** is fixed (stationary arm) and is written to the
  `joint_states` header (`config.base_frame`, default `world`). It must match
  the root link of the deployment URDF in `soma.yaml`.
- `end_pose` / `pos_command` carry unstamped `geometry_msgs/Pose` in the
  **base frame**; positions are the controller's configured TCP pose. The TCP
  definition lives on the controller (user-configured), not in this package.
- Arm joints are revolute, in **rad**, names `joint_1`..`joint_6`.
- The gripper is a prismatic joint `gripper_joint` in **m** (0 = closed,
  `0.098` = fully open; TG-9801 stroke). Adaptive mode is binary: values
  below half stroke close to the configured force, at/above half stroke open.
  This is the **two-finger gap**, the same quantity the standard
  `gripper/state` contract reports. The URDF's `gripper_joint` limit is the
  per-finger travel (`0.049`): the joint stream keeps the gap for the
  deployed consumers (`soma.yaml` `open_position_m: 0.098`, the skill's
  width thresholds), so a URDF viewer shows twice the finger travel unless
  the consumer halves it.
- A TG-9801 register read costs ~1.3 s (single-entry RS485 signal-table
  path), so `gripper_joint` refreshes at ~0.3-0.5 Hz: the last reading is
  republished at the full rate in between and may lag the fingers by a couple
  of seconds. Arm joints are unaffected - the gripper reads hold the RC lock
  only per SDK call, not across their 1.1 s of polling waits.

## Capabilities

| contract | dir | topic | notes |
|---|---|---|---|
| `robonix/primitive/arm/joint_states` | out | `/jaka/joint_states` | reliable; `position[]` populated, `velocity/effort` empty |
| `robonix/primitive/arm/end_pose` | out | `/jaka/end_pose` | ~50 Hz, base frame |
| `robonix/primitive/arm/joint_command` | in | `/jaka/joint_command` | **position-only write**; `velocity/effort` ignored, speed comes from config |
| `robonix/primitive/arm/pos_command` | in | `/jaka/pos_command` | IK on provider: linear move, falls back to controller IK + joint move |
| `robonix/primitive/gripper/state` | out | `/jaka/gripper/state` | `GripperState`; one message per acquisition round (~0.3 Hz) |
| `robonix/primitive/gripper/info` | rpc | `/jaka/gripper/info` | `GetGripperInfo`; static metadata |
| `robonix/primitive/gripper/grip` | rpc | `/jaka/gripper/grip` | `Grip`; adaptive close |
| `robonix/primitive/gripper/open` | rpc | `/jaka/gripper/open` | `Open`; drive to the open endpoint |

`gripper/move` and `gripper/stop` are **not** declared: the TG-9801 has no
verified absolute-width command (force-position mode exists in the firmware
but was never accepted on this controller) and no halt command, so
advertising them would promise semantics the device cannot deliver.

## Gripper service semantics

- `grip` / `open` accept `relative_speed`: `0` means the configured
  `gripper_speed`, `(0,1]` maps onto the device's 200-1500 range.
- `grip` accepts the **adaptive form only**. `relative_effort > 0` is
  rejected with `ok=false` because the closing force is a controller-wide
  setting applied once at connect (`config.gripper_force`, 0-100) and the
  datasheet documents no per-face newton rating; `expected_object_width_valid
  = true` is rejected because width-targeted closing would need move-style
  motion.
- `info` therefore reports `grip_effort_configurable=false`,
  `gripping_force_range_valid=false`,
  `grip_with_expected_object_width_supported=false` and
  `move_speed_configurable=false`, while advertising the 0-0.098 m width
  range and both speed flags.
- Both commands are **rejected while `motion_enabled` is false**, exactly
  like `joint_command` / `pos_command`. `info` is metadata and stays
  available.
- `grip` / `open` return `ok=true` on dispatch, not on arrival or contact -
  the contract says so, and a TG-9801 write only confirms the frame reached
  the controller.

### `gripper/state` semantics

- `opening_width_m` is the measured **two-finger gap**, and
  `opening_width_valid=false` when that register could not be read - the
  last value is never republished as if it were fresh.
- `grasp_state` and `motion_state` both come from the device holding-state
  register (0x0006): 0 idle, 1 moving, 2 holding, 3 slipping. Only 2 becomes
  `HOLDING`; 3 is `SLIPPING`; 0 and 1 are `NOT_HOLDING`. A failed read is
  `UNKNOWN` for both, never inferred from jaw travel or from a command that
  was accepted.
- Measured on this controller (2026-11-12): an empty `grip` reports 1
  (moving) for as long as the firmware keeps the closing force applied, even
  after the jaws reach the stop; `open` returns it to 0 (stopped). Use
  `motion_state` as the device's report, not as a settle/arrival detector.
- `stamp` is the start of the acquisition round, not the publish instant.

## Command semantics

- `joint_command`: subset of `joint_1..joint_6` + optional `gripper_joint`
  allowed; missing arm joints keep their current values. Gripper is driven
  first, then the joint move. Rejected (logged) while `motion_enabled: false`.
- `pos_command`: quaternion converted to the controller's fixed-XYZ RPY
  convention (R = Rz·Ry·Rx). Near j5 singularity or linear failure falls back
  to `kine_inverse` with limit repair.
- Joint targets outside the soft limits are rejected before the SDK call.
- On `CMD_DEACTIVATE` / `CMD_SHUTDOWN` the provider aborts motion; shutdown
  additionally disables and powers off the arm. With `shutdown_home_joints`
  configured, shutdown first opens the gripper, then moves the arm to that
  pose (best-effort, bounded by `move_timeout_s`; see config.spec).

# jaka_arm capability notes

JAKA 6-DOF arm (jkrc SDK over TCP) with a TG-9801 adaptive gripper driven
through the controller's extended-IO channels (a Coboπ Modbus-RTU slave; the
controller's RS485 signal table is unusable on firmware 3.3.15). Provider ID is
the deploy instance `name` (default `jaka_arm`).

## Frames and units

- **Base frame** is fixed (stationary arm) and is written to the
  `joint_states` header (`config.base_frame`, default `world`). It must match
  the root link of the deployment URDF in `soma.yaml`.
- `end_pose` / `pos_command` carry unstamped `geometry_msgs/Pose` in the
  **base frame**; positions are the controller's configured TCP pose. The TCP
  definition lives on the controller (user-configured), not in this package.
- Arm joints are revolute, in **rad**, names `joint_1`..`joint_6`.
- The gripper is a prismatic joint `gripper_joint` in **m**, measuring
  **per-finger travel**: 0 = closed, `0.049` = fully open. Each finger moves
  49 mm, so the two-finger gap is 98 mm. This matches the URDF joint limits,
  where `gripper_joint_r` mirrors `gripper_joint` on the opposite axis.
- `robonix/primitive/gripper/state.opening_width_m` reports the **two-finger
  gap** instead (0-0.098), because that is what the device register holds.
  The relation is `opening_width_m = 2 x gripper_joint`; the provider converts
  at both boundaries. Adaptive mode is binary: values below half travel close
  to the configured force, at/above half travel open.
- Gripper feedback comes from the extended-IO analog inputs (`get_analog_input`,
  one gRPC round trip), so `gripper_joint` refreshes at 5 Hz over a cached
  value. The raw position register is a **closure** amount (1000 = closed,
  0 = open); the provider inverts it against the travel endpoints. Writes go to
  the analog outputs. The Coboπ slave must be running
  (`983126`), the controller must be in **Remote** control source (`10007`), and
  the session must be gRPC: with legacy `login()` the SDK aborts on the first
  ext-IO call. Arm joints are unaffected.
- The slave carries the **full TG-9801 register set**: 11 analog inputs
  (position/current/state, force `Fx/Fy/Fz`, hardness, error flags,
  force-position state, 32-bit grip count) and 9 analog outputs (adaptive
  speed/action, control mode, force-position speed/target, target force, follow
  speed, open/close position). `EXTIO_AI` / `EXTIO_AO` in `jaka_arm/gripper.py`
  are authoritative: if the slave is re-configured in Coboπ, both sides must be
  changed together. Maintenance registers (fingertip calibration/zero, eeprom)
  are deliberately not mapped.

## Capabilities

| contract | dir | topic | notes |
|---|---|---|---|
| `robonix/primitive/arm/joint_states` | out | `/jaka/joint_states` | reliable; `position[]` populated, `velocity/effort` empty |
| `robonix/primitive/arm/end_pose` | out | `/jaka/end_pose` | ~50 Hz, base frame |
| `robonix/primitive/arm/joint_command` | in | `/jaka/joint_command` | **position-only write**; `velocity/effort` ignored, speed comes from config |
| `robonix/primitive/arm/pos_command` | in | `/jaka/pos_command` | IK on provider: linear move, falls back to controller IK + joint move |
| `robonix/primitive/gripper/state` | out | `/jaka/gripper/state` | `GripperState`; one message per acquisition (~5 Hz) |
| `robonix/primitive/gripper/info` | rpc | `/jaka/gripper/info` | `GetGripperInfo`; static metadata |
| `robonix/primitive/gripper/grip` | rpc | `/jaka/gripper/grip` | `Grip`; adaptive close |
| `robonix/primitive/gripper/open` | rpc | `/jaka/gripper/open` | `Open`; drive to the open endpoint |

`gripper/move` and `gripper/stop` are **not** declared: the TG-9801 has no
absolute-width command and no halt command, so advertising them would promise
semantics the device cannot deliver.

### Gripper service semantics

- `open` accepts `relative_speed` only: `0` means the configured
  `gripper_speed`, `(0,1]` maps onto the device's 200-1500 range.
- `grip` accepts the **adaptive form only**. `relative_effort > 0` is rejected
  with `ok=false` because the closing force is not addressable in adaptive mode
  and no per-face newton range is documented; `expected_object_width_valid=true`
  is rejected because width-targeted closing would need move-style motion.
- `info` consequently reports `grip_effort_configurable=false`,
  `gripping_force_range_valid=false`, `grip_with_expected_object_width_supported=false`
  and `move_speed_configurable=false`, while advertising the 0-0.098 m width
  range and both speed flags.
- `state.grasp_state` comes from the device holding register (0x0006);
  `motion_state` is inferred from travel between two acquisitions, so a
  stationary jaw reads `STOPPED` without claiming arrival; `stamp` is the
  acquisition instant, not the publish instant.

### Extended-IO slave recovery

The slave is a controller feature, not an SDK one: while it is down every
analog call returns `-61` and the gripper keeps serving its last cached value,
so the provider no longer fails silently.

On connect, if `is_extio_running()` is false it logs into the App
(`web_user` / `web_password`), captures the control source, posts
`MSGID_SETEXTIOMODE` with `{submod, tarmode: 1}` — the App's own payload, since
the bare `intid` form is silently ignored — waits for the slave to report
running, hands the control source back to Remote, and only then enables the arm.

The controller enforces two preconditions: the arm must be powered and
**disarmed** (the App refuses to start a slave while the arm is enabled), and
the App session must be free (a logged-in App makes the login fail with 917508).
`extio_autostart: false` disables the attempt; connect then fails with the
reason instead of guessing.

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

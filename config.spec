config:
  # string, required, no default.
  # JAKA controller IP address (jkrc manages its own TCP ports).
  ip: 10.5.5.100

  # bool, default: false. SAFETY GATE.
  # When false, joint_command / pos_command and the gripper grip/open services
  # are rejected with a rate-limited warning; only feedback is published.
  # Follow the staged acceptance flow: boot with false (read-only arm), verify
  # feedback and estop, then enable.
  motion_enabled: false

  # float, rad/s, default: 1.0; must be > 0.
  # Joint move speed passed to jkrc joint_move (100% ≈ 180 deg/s hardware max).
  joint_speed_rad_s: 1.0

  # float, mm/s, default: 100.0; must be > 0.
  # Cartesian linear_move speed.
  linear_speed_mm_s: 100.0

  # int, 0..100, default: 50.
  # TG-9801 target grip force (adaptive mode), applied once at connect. The
  # device has no per-face newton rating, so the standard gripper/Grip contract
  # rejects a per-command relative_effort; change the force here instead.
  gripper_force: 50

  # int, 200..1500, default: 1100.
  # TG-9801 adaptive grip/release speed, used by joint_command's gripper_joint
  # and by gripper/grip|open when relative_speed = 0.
  gripper_speed: 1100

  # float, deg^2, default: 1.0.
  # Mean-squared joint error below which a move counts as reached.
  reach_mse_threshold_deg2: 1.0

  # float, seconds, default: 10.0; must be > 0.
  # Timeout for joint/pose move completion waits.
  move_timeout_s: 10.0

  # list of 6 floats (rad), optional, default: unset (no motion on exit).
  # On provider shutdown (Ctrl-C / `rbnx shutdown`), the gripper opens first
  # (releasing any held object), then the six arm joints move to this pose,
  # before the disconnect/power-off sequence — so the arm parks instead of
  # stopping wherever it was commanded to last. Best-effort: bounded by
  # move_timeout_s, failures are logged and power-off proceeds. Skipped
  # while motion_enabled is false.
  # shutdown_home_joints: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

  # float, Hz, default: 50.0.
  # joint_states / end_pose publish rate. gripper/state is not covered: it is
  # published once per acquisition round (~0.3 Hz; one round costs ~2.6 s on
  # the RS485 signal-table path), and arm feedback above runs independently.
  state_publish_hz: 50.0

  # string, default: world.
  # Frame id written to the joint_states header. Must match the root link of
  # the deployment's whole-robot URDF (soma.yaml urdf.root_link).
  base_frame: world

  # map, optional. Overrides for the four ROS 2 topic names. Keys:
  # joint_states / joint_command / pos_command / end_pose / gripper_state.
  # topics: { joint_states: /jaka/joint_states }

  # map, optional. Overrides for the three gripper ROS 2 service names. Keys:
  # info / grip / open.
  # services: { grip: /jaka/gripper/grip }

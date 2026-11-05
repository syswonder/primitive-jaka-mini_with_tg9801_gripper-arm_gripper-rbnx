config:
  # string, required, no default.
  # JAKA controller IP address (jkrc manages its own TCP ports).
  ip: 10.5.5.100

  # bool, default: false. SAFETY GATE.
  # When false, joint_command / pos_command are rejected with a rate-limited
  # warning; only feedback is published. Follow the staged acceptance flow:
  # boot with false (read-only arm), verify feedback and estop, then enable.
  motion_enabled: false

  # float, rad/s, default: 1.0; must be > 0.
  # Joint move speed passed to jkrc joint_move (100% ≈ 180 deg/s hardware max).
  joint_speed_rad_s: 1.0

  # float, mm/s, default: 100.0; must be > 0.
  # Cartesian linear_move speed.
  linear_speed_mm_s: 100.0

  # int, 0..100, default: 50.
  # TG-9801 target grip force (adaptive mode).
  gripper_force: 50

  # int, 200..1500, default: 1100.
  # TG-9801 adaptive grip/release speed.
  gripper_speed: 1100

  # string, default: jaka_sdk.
  # SDK account used for the gRPC login. The extended-IO gripper channels only
  # exist on a gRPC session.
  # str, default: jakazu7admin. App account the provider logs in with when it
  # has to start the controller's extended-IO slave by itself.
  web_user: jakazu7admin

  # str, default: "". App password for `web_user`. Empty disables automatic
  # slave recovery (see extio_autostart) and keeps the credential out of files.
  web_password: ""

  # bool, default: true. When the extended-IO slave is down at connect time,
  # capture the control source, start the slave and hand control back. With
  # this false the provider fails connect instead of trying.
  extio_autostart: true

  # float, s, default: 10.0. How long to wait for the slave to report running.
  extio_start_timeout_s: 10.0

  sdk_user: jaka_sdk

  # string, required for the gRPC session on the Lumi controller; there is no
  # built-in default. Keep the value in the machine-local deployment manifest
  # (gitignored), never in this file.
  # Leave empty to fall back to the legacy login(); the gripper is then
  # disabled, because its ext-IO SDK calls abort the process on a legacy
  # session.
  sdk_password: ""

  # int, default: 54052.
  # Coboπ Modbus-slave submodule id that backs the gripper's extended-IO
  # channels (settings -> hardware & communication -> Modbus -> the TIO Modbus
  # RTU slave). The slave must be running and the controller must be in Remote
  # control source, otherwise every write returns 10007 and reads 10003.
  gripper_submodule_id: 54052

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
  # joint_states / end_pose publish rate.
  state_publish_hz: 50.0

  # string, default: world.
  # Frame id written to the joint_states header. Must match the root link of
  # the deployment's whole-robot URDF (soma.yaml urdf.root_link).
  base_frame: world

  # map, optional. Overrides for the four ROS 2 topic names. Keys:
  # joint_states / joint_command / pos_command / end_pose.
  # topics: { joint_states: /jaka/joint_states }

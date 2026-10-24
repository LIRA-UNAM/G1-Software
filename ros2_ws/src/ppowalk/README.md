# ppowalk

Runs velocity-tracking (walking) policies trained with
[mjlab-biped-playground](../../../../mjlab-biped-playground)
(`Pumas-Velocity-{Flat,Rough}-Unitree-G1-*`, IMU-only actor) on the 23-DoF G1.
It reuses getup's `g1_lowlevel_bridge` (gains, joint limits, offline motors,
motor mapping) and its ONNX / observation helpers.

```bash
make ppowalk                                   # uses ~/ppowalk_models/g1_ppowalk.onnx
make ppowalk WALK_POLICY=/abs/path/model.onnx WALK_ARGS="enable_lowcmd:=true"
# GUI: http://<robot-ip>:8084
```

Every `*.onnx` in the models folder (`models_dir`; by default the folder of the
launched model, i.e. `~/ppowalk_models` with `make ppowalk`) is offered in the
GUI's Model dropdown. Choosing one while the policy is stopped loads it for the
next Start. Choosing one while the policy runs switches to it live, with no
Home / Start needed. This is allowed only at a standstill: zero requested
velocity, body angular velocity <= `switch_max_ang_vel` (0.3 rad/s) and leg
joint velocities <= `switch_max_joint_vel` (0.5 rad/s), continuously for
`switch_settle_s` (0.5 s). The new policy blends in over `start_ramp_s`, like
Start. Otherwise the dropdown is disabled with the reason, and the node refuses
a `policy_path` change. A model that is not 67 -> 12 is rejected and the
current one stays loaded.

## What the node reproduces from training

* Observation (67): `base_ang_vel` (3, pelvis gyro), `projected_gravity` (3),
  `joint_pos - default` (23), `joint_vel` (23), last raw action (12),
  velocity command `(vx, vy, wz)` (3).
* Action: the 12 leg joints, absolute `target = default_joint_pos + action_scale * a`.
  Policy at 50 Hz.
* Commanded joints (`command_joints`, default legs + waist) are published; the
  bridge damps the rest. `always_commanded_joints` (the waist) can never be
  dropped, so the waist stays stiff at 0 whenever the node commands the robot.
  (The policy's IMU is the pelvis IMU, as in training; the waist matters for the
  torso dynamics.) Commanded joints the policy does not drive are held at
  `held_joint_targets` = 0: the training env holds waist and arms at 0, not at
  the default pose.
* Pinned observations: non-commanded joints, motors detected offline (latched
  for the run) and `obs_pinned_joints` are reported to the policy at their hold
  target, at rest. In simulation, a limp arm with real (or flapping) readings
  makes the policy fall; pinned, it walks. Start is refused while a leg motor is
  offline.
* All values live in `config/g1_ppowalk_policy.yaml` and can be changed from the GUI.

## Controls

* **Upright gate:** the node always computes the tilt from the IMU. Start is
  refused (and disabled in the GUI) unless the tilt is at most
  `max_start_tilt_deg` (20 deg). The gate only applies to starting.
* **Home:** slow cosine move of the commanded joints (peak `home_max_vel`, at
  least `home_min_duration_s`) to `home_pose` (zero pose by default; editable,
  or "Home = default pose" in the GUI), then hold.
* **Start** (GUI button or Start on the wireless remote): blends from the
  current pose into the policy targets over `start_ramp_s`, then runs the policy.
* **Stop:** stops the policy and holds the current pose (stays stiff).
* **Release:** stops commanding; the bridge damps the joints (a standing robot
  collapses).
* **Velocity command:** one source at a time, chosen with the GUI's "Command
  source" switch: the wireless remote's sticks (default, `use_remote`), the GUI
  joysticks (`cmd_vx/cmd_vy/cmd_wz`; left = vx/vy, right = wz, releasing a stick
  sends 0), or `geometry_msgs/Twist` on `/cmd_vel`
  (`use_cmd_vel`; falls back to 0 if it stops for `cmd_vel_timeout_s`). Every
  source is clamped to the limits (±0.7 m/s, ±0.7 m/s, ±0.7 rad/s by default).
  Calibration offsets (`cmd_vx_offset/cmd_vy_offset/cmd_wz_offset`) are added
  to cancel the real robot's drift, only while motion is commanded unless
  `offsets_when_standing`.
* If IMU or joint data go stale while running, the node holds the last pose.
  There is no software emergency stop: keep the remote's L2+B (damping) at hand.

## Wireless remote

The bridge republishes the Unitree remote (`LowState.wireless_remote`) as
`sensor_msgs/Joy` on `/wireless_remote` (50 Hz; axes `[lx, ly, rx, ry, L2]`,
buttons in Unitree bit order `R1, L1, Start, Select, R2, L2, F1, F2, A, B, X, Y,
up, right, down, left`). The GUI's command panel shows the live sticks and
buttons, which is a quick check that the remote is linked.

* **Sticks** (`use_remote: true`): left stick up/down = vx, left/right = vy,
  right stick left/right = wz (turn). Full deflection reaches the velocity
  limits; within `remote_deadzone` (0.1) the command is exactly 0, so letting go
  of the sticks makes the robot stand still. If the remote data stops for
  `remote_timeout_s` the command falls back to 0.
* **Start** (`remote_start: true`): a press runs the same Start as the GUI,
  with every check: robot upright (tilt <= `max_start_tilt_deg`), leg motors
  online, fresh sensor data, model loaded. A refused press is logged and shown
  in the GUI ("Last remote event"). Only the press counts; holding the button
  does not repeat it, and it does nothing while the policy runs.
* There is no stop button on the remote: use the GUI's Stop / Release, or the
  remote's L2+B (damping).
* A started policy only reaches the motors with motor output enabled
  (`enable_lowcmd`, robot in debug mode with the built-in controller released).

Typical run: Home from the GUI with the robot hanging, lower it until it stands
upright, press Start on the remote, steer with the sticks.

## Export a policy

In `mjlab-biped-playground`:

```bash
uv run python export_policy.py Pumas-Velocity-Flat-Unitree-G1-PPO \
    --wandb-run-path Pumanoids/g1_velocity/<run> \
    --export-dir export --filename g1_velocity_flat_ppo_<run>.onnx
scp export/g1_velocity_flat_ppo_<run>.onnx g1:ppowalk_models/   # shows up in the dropdown
```

The node checks the model is 67 -> 12 and refuses anything else.

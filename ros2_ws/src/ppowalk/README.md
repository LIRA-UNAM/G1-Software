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
* **Start:** blends from the current pose into the policy targets over
  `start_ramp_s`, then runs the policy.
* **Stop:** stops the policy and holds the current pose (stays stiff).
* **Release:** stops commanding; the bridge damps the joints (a standing robot
  collapses).
* **Velocity command:** GUI sliders (`cmd_vx/cmd_vy/cmd_wz`), clamped to the
  limits (±0.7 m/s, ±0.7 m/s, ±0.7 rad/s by default). Calibration offsets
  (`cmd_vx_offset/cmd_vy_offset/cmd_wz_offset`) are added to cancel the real
  robot's drift, only while motion is commanded unless `offsets_when_standing`; or `use_cmd_vel: true` to take `geometry_msgs/Twist` from
  `/cmd_vel` (falls back to 0 if it stops for `cmd_vel_timeout_s`).
* If IMU or joint data go stale while running, the node holds the last pose.
  There is no software emergency stop: keep the remote's L2+B (damping) at hand.

## Export a policy

In `mjlab-biped-playground`:

```bash
uv run python export_policy.py Pumas-Velocity-Flat-Unitree-G1-PPO \
    --wandb-run-path Pumanoids/mjlab_playground/<run> \
    --export-dir export --filename g1_ppowalk.onnx
```

The node checks the model is 67 -> 12 and refuses anything else.

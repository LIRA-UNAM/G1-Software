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
* Action: the 12 leg joints, absolute `target = default_joint_pos + action_scale * a`;
  waist and arms are held at `default_joint_pos`. Policy at 50 Hz.
* All values live in `config/g1_ppowalk_policy.yaml` and can be changed from the GUI.

## Controls

* **Upright gate:** the node always computes the tilt from the IMU. Start is
  refused (and disabled in the GUI) unless the tilt is at most
  `max_start_tilt_deg` (30 deg). The gate only applies to starting.
* **Home:** slow cosine move (peak `home_max_vel`, at least
  `home_min_duration_s`) to `home_pose` (zero pose by default; editable, or
  "Home = default pose" in the GUI), then hold.
* **Start:** blends from the current pose into the policy targets over
  `start_ramp_s`, then runs the policy.
* **Stop:** stops the policy and holds the current pose (stays stiff).
* **Release:** stops commanding; the bridge damps the joints (a standing robot
  collapses).
* **Velocity command:** GUI sliders (`cmd_vx/cmd_vy/cmd_wz`), clamped to the
  limits; or `use_cmd_vel: true` to take `geometry_msgs/Twist` from
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

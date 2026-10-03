# getup

Runs fall-recovery (getup) policies trained with
[mjlab-biped-playground](../../../../mjlab-biped-playground) (task
`Pumas-Getup-Flat-Unitree-G1`) on the Unitree G1, using ONNX Runtime from C++.

Two nodes:

| Node | In | Out |
|------|----|-----|
| `g1_lowlevel_bridge` | `/lowstate` (`unitree_hg/LowState`), `/getup/joint_command` (`sensor_msgs/JointState`) | `/imu` (`sensor_msgs/Imu`), `/joint_states`, `/lowcmd` (`unitree_hg/LowCmd`, Kp/Kd + CRC) |
| `getup_policy_node` | `/imu`, `/joint_states` | `/getup/joint_command` (position targets, policy joint order), `~/observation`, `~/action` (debug) |

The policy node only depends on generic messages, so it can also be driven by
a simulator that publishes `/imu` and `/joint_states` (`use_bridge:=false`).

## What the node reproduces from training

* Observation (75 for the 23-DoF G1): `base_ang_vel` (pelvis gyro, 3),
  `projected_gravity` (3, from the pelvis IMU quaternion),
  `joint_pos - default_joint_pos` (23), `joint_vel` (23), last raw action (23).
* Policy at 50 Hz (sim dt 0.005 x decimation 4).
* Relative action `target = q + 0.6 * action`, re-evaluated at 200 Hz with the
  latest joint positions (mjlab applies it every physics substep).
* Settle phase: for the first `settle_steps` (50 = 1 s) policy steps after
  `~/start`, the target is the current position. The policy still runs and its
  output feeds the last-action observation.
* PD gains of the training actuators (bridge `kp`/`kd`).

All hyperparameters are ROS parameters in `config/g1_getup_policy.yaml` and
`config/g1_lowlevel_bridge.yaml`.

## Robot variant: 23-DoF G1

Our G1 is the 23-DoF version: waist roll/pitch and wrist pitch/yaw do not
exist (their LowState slots 13, 14, 20, 21, 27, 28 report 0 V). Policies must be
trained with the 23-DoF `g1.xml`. The configs list the 23 joints, and the
bridge maps them onto the 35-slot LowCmd/LowState with `motor_indices`
(`[0..12, 15..19, 22..26]`). Unused slots get mode 0 and zero gains.

## Emergency stop and GUI

`ros2 launch surge_et_ambula getup.launch.py policy_path:=...` (or
`make getup` from the repository root) starts the bridge, the policy and the
`getup_gui` web dashboard at `http://<robot-ip>:8082`.

* The e-stop is triggered by the GUI button, Space or Esc on the page,
  `ros2 service call /g1_lowlevel_bridge/estop std_srvs/srv/Trigger`, or
  `true` on `/getup/estop`. It latches the bridge into damping (`kp=0`,
  `kd=damping_kd`) and stops the policy. Only `~/reset_estop` (the GUI Reset
  button) clears it. The robot then stays in damping until the policy is
  started again.
* Dead-man: the GUI sends a heartbeat on `/getup/heartbeat` about 10 times per
  second. If motor output is enabled and no heartbeat arrives for
  `heartbeat_timeout_s` (0.5 s; armed by the first heartbeat, 0 disables it),
  the bridge latches the e-stop. This covers a closed tab, lost Wi-Fi or a
  crashed GUI.
* All non-structural parameters (gains, limits, timeouts, action scale/clip,
  settle steps, rates, default pose, observation scales, model path while
  stopped) can be changed live from the GUI or with `ros2 param set`.
* Limitation: if the bridge process itself dies, the LowCmd stream stops and
  no software e-stop can act. Keep the remote's L2+B (damping) at hand, and run
  the launch inside tmux so an SSH drop does not kill it.

## Build

```bash
cd ros2_ws
colcon build --packages-select unitree_hg getup
source install/setup.bash
```

The prebuilt ONNX Runtime (x64 or aarch64) is downloaded at configure time
and installed with the package. Pass `--cmake-args -DONNXRUNTIME_ROOT=/path`
to use a local install instead.

## Export a policy

In `mjlab-biped-playground`:

```bash
uv run python export_policy.py Pumas-Getup-Flat-Unitree-G1 \
    --checkpoint-file logs/rsl_rl/g1_getup/wandb_checkpoints/<run>/model_2999.pt \
    --export-dir export --filename g1_getup.onnx
```

The ONNX metadata (`joint_names`, `default_joint_pos`, `joint_stiffness`,
`joint_damping`, `observation_names`, `action_scale`, `action_type=relative`)
should match the YAML files. Inspect it with:

```bash
python -c "import onnx; [print(p.key, '=', p.value) for p in onnx.load('export/g1_getup.onnx').metadata_props]"
```

## Run

1. Dry run (no motor commands, `enable_lowcmd` is false by default):
   ```bash
   ros2 launch getup getup.launch.py policy_path:=/abs/path/g1_getup.onnx
   ros2 topic hz /imu /joint_states
   ```
2. On the robot (suspended on a gantry for the first tests):
   * Put the G1 in **debug mode** (high-level motion control released, e.g.
     L2+R2 on the remote from damping mode). Otherwise the built-in locomotion
     controller fights the low-level commands.
   * Enable low-level output. With no joint command the bridge streams
     damping (`kp=0`, `kd=damping_kd`):
     ```bash
     ros2 param set /g1_lowlevel_bridge enable_lowcmd true
     ```
     (or launch with `enable_lowcmd:=true`).
   * Start / stop the policy:
     ```bash
     ros2 service call /getup_policy_node/start std_srvs/srv/Trigger
     ros2 service call /getup_policy_node/stop std_srvs/srv/Trigger
     ```

Safety behaviour:
* Stale IMU or joint data (older than `data_timeout_s`) stops the policy.
* The bridge damps when no command has arrived within `command_timeout_s`.
* The bridge clamps targets to the joint ranges from `g1.xml`.
  `max_target_delta` optionally limits `|target - q|`. Training used no
  action clipping, and getup policies can output large relative targets, so
  a tight limit changes the trained behaviour.

## Tests

```bash
colcon test --packages-select getup && colcon test-result --verbose
# Also check an exported policy (93 -> 29):
GETUP_POLICY_PATH=/abs/path/g1_getup.onnx ./build/getup/test_onnx_policy
```

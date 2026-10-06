"""Records joint motions at a fixed rate and replays them through g1_lowlevel_bridge.

Recording (teach mode): the bridge streams damping at a low `damping_kd` while
no joint command is published, so the limbs can be guided by hand; this node
samples /joint_states at `record_rate_hz` and saves the take as a .pkl file.

Playback: safe cosine move to the recording's first frame, real-time replay of
the selected joint groups (others hold), then a safe move back to the first
frame and hold. Targets are published on the bridge's command topic, so the
bridge keeps owning gains, joint limits, the e-stop latch and the dead-man.
"""

import datetime
import json
import os
import re
import signal
import time

import numpy as np
from rcl_interfaces.msg import ParameterType, SetParametersResult
from rcl_interfaces.srv import GetParameters, SetParameters
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from . import storage
from .trajectory import (interp_frames, reorder, safe_move_duration, smooth_interp,
                         validate_recording)

IDLE = "IDLE"
RECORDING = "RECORDING"
GOING_HOME = "GOING_HOME"
APPROACHING = "APPROACHING"
PLAYING = "PLAYING"
PAUSED = "PAUSED"
RETURNING = "RETURNING"
HOLDING = "HOLDING"
ESTOP = "ESTOP"
# States in which this node owns the joints and streams targets.
COMMANDING = (GOING_HOME, APPROACHING, PLAYING, PAUSED, RETURNING, HOLDING)
# Safe (speed-limited) moves between keyframes.
SEGMENT_STATES = (GOING_HOME, APPROACHING, RETURNING)
# Skip the move to home when every played joint is already this close (rad).
AT_HOME_TOLERANCE = 0.02

DEFAULT_GROUPS = [
    "legs:.*_(hip|knee|ankle)_.*",
    "waist:waist_.*",
    "left_arm:left_(shoulder|elbow|wrist)_.*",
    "right_arm:right_(shoulder|elbow|wrist)_.*",
]


class MotionRecorder(Node):
    def __init__(self):
        super().__init__("motion_recorder")
        self.declare_parameter("recordings_dir", os.path.expanduser("~/motion_recordings"))
        self.declare_parameter("record_rate_hz", 20.0)
        self.declare_parameter("command_rate_hz", 100.0)
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("command_topic", "/getup/joint_command")
        self.declare_parameter("estop_topic", "/getup/estop")
        self.declare_parameter("bridge_node", "/g1_lowlevel_bridge")
        self.declare_parameter("approach_max_vel", 0.3)
        self.declare_parameter("approach_min_duration_s", 2.0)
        self.declare_parameter("max_recorded_vel", 4.0)
        self.declare_parameter("data_timeout_s", 0.2)
        self.declare_parameter("joint_groups", DEFAULT_GROUPS)
        # Live-editable arguments of the services (set by the GUI).
        self.declare_parameter("teach_damping_kd", 0.3)
        self.declare_parameter("recording_name", "")
        self.declare_parameter("selected_recording", "")
        self.declare_parameter("play_groups", ["legs", "waist", "left_arm", "right_arm"])
        self.declare_parameter("record_groups", ["legs", "waist", "left_arm", "right_arm"])

        p = lambda name: self.get_parameter(name).value  # noqa: E731
        self.recordings_dir = os.path.expanduser(p("recordings_dir"))
        self.record_rate = float(p("record_rate_hz"))
        self.command_rate = float(p("command_rate_hz"))
        self.groups = self._parse_groups(p("joint_groups"))
        self.index = storage.RecordingIndex(self.recordings_dir)
        os.makedirs(self.recordings_dir, exist_ok=True)

        self.state = IDLE
        self.message = "ready"
        self.joint_names = None  # bridge joint order, from the first /joint_states
        self.q = self.dq = self.tau = None
        self.last_js = None
        self.target = None  # last published target (np array, bridge order)
        self.buffer = None  # recording in progress
        self.last_saved = None  # path of the last saved take (for discard)
        self.rec = None  # loaded recording for playback
        self.rec_q = None  # recording q reordered to bridge order
        self.selected_mask = None
        self.segment = None  # (q_from, q_to, t0, duration) for safe moves
        self.play_clock = 0.0
        self.play_last = None
        self.saved_damping_kd = None
        self.recordings = []
        self._listing_time = 0.0
        self.home = storage.load_home(self.recordings_dir)  # persisted home keyframe
        self.pending = []  # queued (q_to, state) safe moves
        self.offline = set()  # joints the bridge reports offline (no voltage / fault)
        self.bridge_status_time = None

        reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(JointState, p("joint_states_topic"), self._on_joint_state,
                                 qos_profile_sensor_data)
        self.create_subscription(Bool, p("estop_topic"), self._on_estop, reliable)
        self.estop_pub = self.create_publisher(Bool, p("estop_topic"), reliable)
        self.command_pub = self.create_publisher(JointState, p("command_topic"), 10)
        self.status_pub = self.create_publisher(String, "~/status", 10)

        bridge = p("bridge_node")
        self.create_subscription(String, bridge + "/status", self._on_bridge_status, 10)
        self.bridge_get = self.create_client(GetParameters, bridge + "/get_parameters")
        self.bridge_set = self.create_client(SetParameters, bridge + "/set_parameters")
        self.bridge_reset = self.create_client(Trigger, bridge + "/reset_estop")

        for name, handler in (("record_start", self.record_start),
                              ("record_stop", self.record_stop),
                              ("discard", self.discard), ("play", self.play),
                              ("pause", self.pause), ("resume", self.resume),
                              ("reset", self.reset), ("release", self.release),
                              ("set_home", self.set_home)):
            self.create_service(Trigger, "~/" + name, self._wrap(handler))

        self.add_on_set_parameters_callback(self._on_set_parameters)
        self.record_timer = None
        self.create_timer(1.0 / self.command_rate, self._command_step)
        self.create_timer(0.1, self._publish_status)
        self.get_logger().info(
            "motion_recorder ready: %s, record %.0f Hz, command %.0f Hz, groups %s"
            % (self.recordings_dir, self.record_rate, self.command_rate, list(self.groups)))

    # ------------------------------------------------------------ parameters

    @staticmethod
    def _parse_groups(entries):
        groups = {}
        for entry in entries:
            name, _, pattern = entry.partition(":")
            if not name or not pattern:
                raise ValueError("joint_groups entries must be 'name:regex', got %r" % entry)
            groups[name] = re.compile(pattern)
        return groups

    def _on_set_parameters(self, params):
        for prm in params:
            v = prm.value
            if prm.name == "teach_damping_kd" and (not isinstance(v, (int, float)) or v < 0.0):
                return SetParametersResult(successful=False, reason="teach_damping_kd must be >= 0")
            if prm.name == "recording_name" and v and not storage.valid_name(v):
                return SetParametersResult(
                    successful=False, reason="recording_name: 1-64 chars of A-Z a-z 0-9 _ -")
            if prm.name == "selected_recording" and v:
                if os.path.basename(v) != v or not v.endswith(".pkl"):
                    return SetParametersResult(successful=False,
                                               reason="selected_recording must be a .pkl file name")
                if not os.path.exists(os.path.join(self.recordings_dir, v)):
                    return SetParametersResult(successful=False, reason="no such recording: " + v)
            if prm.name in ("play_groups", "record_groups"):
                unknown = [g for g in (v or []) if g not in self.groups]
                if unknown or not v:
                    return SetParametersResult(
                        successful=False,
                        reason="%s must be a non-empty subset of %s" % (prm.name, list(self.groups)))
            if prm.name in ("recordings_dir", "record_rate_hz", "command_rate_hz", "joint_groups",
                            "joint_states_topic", "command_topic", "estop_topic", "bridge_node"):
                return SetParametersResult(successful=False, reason=prm.name + " is read-only")
        return SetParametersResult(successful=True)

    def _p(self, name):
        return self.get_parameter(name).value

    # ---------------------------------------------------------------- inputs

    def _on_joint_state(self, msg):
        if len(msg.position) != len(msg.name):
            return
        if self.joint_names != list(msg.name):
            if self.state in COMMANDING or self.state == RECORDING:
                self._trigger_estop("joint_states names changed")
            self.joint_names = list(msg.name)
        self.q = np.asarray(msg.position, float)
        n = len(msg.name)
        self.dq = np.asarray(msg.velocity, float) if len(msg.velocity) == n else np.zeros(n)
        self.tau = np.asarray(msg.effort, float) if len(msg.effort) == n else np.zeros(n)
        self.last_js = time.monotonic()

    def _on_bridge_status(self, msg):
        try:
            self.offline = set(json.loads(msg.data).get("offline_joints", []))
            self.bridge_status_time = time.monotonic()
        except ValueError:
            pass

    def _group_mask(self, groups):
        """Boolean mask over self.joint_names for the given group names."""
        return np.array([any(self.groups[g].fullmatch(n) for g in groups)
                         for n in self.joint_names])

    def _offline_groups(self):
        return [g for g, rx in self.groups.items() if any(rx.fullmatch(n) for n in self.offline)]

    def _data_fresh(self):
        return self.last_js is not None and time.monotonic() - self.last_js < self._p("data_timeout_s")

    def _on_estop(self, msg):
        if not msg.data or self.state == ESTOP:
            return
        if self.state == RECORDING:
            self._finish_recording(save=False)
        self.get_logger().error("E-STOP received: stopping (%s)" % self.state)
        self.state = ESTOP
        self.message = "e-stop latched: press Reset"
        self.segment = None
        self.pending = []

    def _trigger_estop(self, reason):
        self.get_logger().error("Triggering e-stop: " + reason)
        self.estop_pub.publish(Bool(data=True))
        self._on_estop(Bool(data=True))
        self.message = "e-stop: " + reason

    # -------------------------------------------------------------- services

    def _wrap(self, handler):
        def callback(_request, response):
            try:
                response.message = handler() or "ok"
                response.success = True
            except (RuntimeError, ValueError, OSError) as exc:
                response.success = False
                response.message = str(exc)
                self.get_logger().warning("%s refused: %s" % (handler.__name__, exc))
            return response
        return callback

    def record_start(self):
        if self.state not in (IDLE, HOLDING):
            raise RuntimeError("cannot record while %s" % self.state)
        if not self._data_fresh():
            raise RuntimeError("no fresh /joint_states")
        groups = list(self._p("record_groups"))
        index = np.flatnonzero(self._group_mask(groups))
        if not len(index):
            raise RuntimeError("record_groups select no joints")
        # Stop commanding so the bridge streams (teach) damping.
        self.state = RECORDING
        self.target = None
        self.buffer = {"t0": time.monotonic(), "t": [], "q": [], "dq": [], "tau": [], "online": [],
                       "index": index, "names": [self.joint_names[i] for i in index],
                       "groups": groups,
                       "created": datetime.datetime.now().isoformat(timespec="seconds")}
        self._set_teach_damping()
        self.record_timer = self.create_timer(1.0 / self.record_rate, self._record_step)
        self._record_step()
        self.message = "recording"
        offline = sorted(set(self.buffer["names"]) & self.offline)
        warn = " (WARNING offline: %s)" % ", ".join(offline) if offline else ""
        return "recording %s at %.0f Hz%s" % ("+".join(groups), self.record_rate, warn)

    def record_stop(self):
        if self.state != RECORDING:
            raise RuntimeError("not recording")
        path = self._finish_recording(save=True)
        return "saved " + os.path.basename(path)

    def discard(self):
        if self.state == RECORDING:
            self._finish_recording(save=False)
            self.message = "recording discarded"
            return self.message
        if self.state in COMMANDING:
            raise RuntimeError("cannot discard while %s" % self.state)
        if not self.last_saved or not os.path.exists(self.last_saved):
            raise RuntimeError("no take to discard")
        name = os.path.basename(self.last_saved)
        os.unlink(self.last_saved)
        self.last_saved = None
        self._listing_time = 0.0  # refresh the dropdown now
        if self._p("selected_recording") == name:
            self.set_parameters([Parameter("selected_recording", Parameter.Type.STRING, "")])
        self.message = "deleted " + name
        return self.message

    def set_home(self):
        """Home keyframe = current measured joint positions (online motors only)."""
        if self.state not in (IDLE, HOLDING, ESTOP):
            raise RuntimeError("cannot set home while %s" % self.state)
        if self.joint_names is None or not self._data_fresh():
            raise RuntimeError("no fresh /joint_states")
        online = [n not in self.offline for n in self.joint_names]
        self.home = storage.save_home(
            self.recordings_dir, self.joint_names, self.q, online,
            datetime.datetime.now().isoformat(timespec="seconds"))
        skipped = [n for n, ok in zip(self.joint_names, online) if not ok]
        self.message = "home set (%d joints)%s" % (
            len(self.home["joints"]), "; offline, not stored: " + ", ".join(skipped) if skipped else "")
        self.get_logger().info(self.message)
        return self.message

    def play(self):
        if self.state not in (IDLE, HOLDING):
            raise RuntimeError("cannot play while %s" % self.state)
        self._load_selected()
        start = self._start_pose()
        home = self._home_pose(start)
        self.play_clock = 0.0
        self.play_last = None
        # home -> start of the take -> play -> home (or start -> play -> start).
        if home is not None and not self._near(home):
            self._begin_segment(home, GOING_HOME)
            self.pending = [(start, APPROACHING)]
            return "moving to home, then to the start pose"
        self.pending = []
        self._begin_segment(start, APPROACHING)
        return "moving to start pose (%.1f s)" % self.segment[3]

    def pause(self):
        if self.state != PLAYING:
            raise RuntimeError("not playing")
        self.state = PAUSED
        self.message = "paused at %.1f s" % self.play_clock
        return self.message

    def resume(self):
        if self.state != PAUSED:
            raise RuntimeError("not paused")
        self.state = PLAYING
        self.play_last = time.monotonic()
        self.message = "playing"
        return "resumed"

    def reset(self):
        if self.state == RECORDING:
            raise RuntimeError("stop the recording first")
        if not self.bridge_reset.service_is_ready():
            raise RuntimeError("bridge reset_estop service not available")
        future = self.bridge_reset.call_async(Trigger.Request())
        future.add_done_callback(self._after_bridge_reset)
        self.message = "resetting e-stop"
        return "reset requested"

    def _after_bridge_reset(self, future):
        result = future.result()
        if result is None or not result.success:
            self.message = "bridge e-stop reset failed"
            return
        self.state = IDLE
        self.segment = None
        self.pending = []
        if not self._p("selected_recording"):
            self.message = "e-stop cleared (no recording selected)"
            return
        try:
            self._load_selected()
            self.target = None  # start from the measured (damped) pose
            start = self._start_pose()
            home = self._home_pose(start)
            self._begin_segment(home if home is not None else start, RETURNING)
            self.message = "e-stop cleared: returning to %s" % (
                "home" if home is not None else "start pose")
        except (RuntimeError, ValueError, OSError) as exc:
            self.message = "e-stop cleared; cannot return: %s" % exc

    def release(self):
        if self.state not in COMMANDING:
            raise RuntimeError("nothing to release")
        self.state = IDLE
        self.segment = None
        self.pending = []
        self.target = None
        self.message = "released (bridge damping)"
        return self.message

    # ------------------------------------------------------------- recording

    def _record_step(self):
        if self.state != RECORDING:
            return
        if not self._data_fresh():
            self._finish_recording(save=False)
            self.message = "recording aborted: /joint_states timed out"
            self.get_logger().error(self.message)
            return
        b = self.buffer
        i = b["index"]
        b["t"].append(time.monotonic() - b["t0"])
        b["q"].append(self.q[i].copy())
        b["dq"].append(self.dq[i].copy())
        b["tau"].append(self.tau[i].copy())
        # Saved as-is; playback refuses joints that were offline in the take.
        b["online"].append([n not in self.offline for n in b["names"]])

    def _finish_recording(self, save):
        if self.record_timer is not None:
            self.destroy_timer(self.record_timer)
            self.record_timer = None
        self._restore_damping()
        b, self.buffer = self.buffer, None
        self.state = IDLE
        if not save:
            return None
        if b is None or len(b["t"]) < 2:
            raise RuntimeError("recording too short")
        name = self._p("recording_name") or datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        rec = storage.make_recording(b["names"], self.record_rate, b["t"], b["q"], b["dq"],
                                     b["tau"], b["created"], online=b["online"],
                                     groups=b["groups"])
        path = storage.save_recording(self.recordings_dir, name, rec)
        self.last_saved = path
        self._listing_time = 0.0  # refresh the dropdown now
        self.message = "saved %s (%d frames, %.1f s)" % (
            os.path.basename(path), len(b["t"]), rec["duration_s"])
        self.get_logger().info(self.message)
        self.set_parameters([Parameter("selected_recording", Parameter.Type.STRING,
                                       os.path.basename(path))])
        return path

    def _set_teach_damping(self):
        # Remember the bridge's damping, then lower it for teaching.
        if not (self.bridge_get.service_is_ready() and self.bridge_set.service_is_ready()):
            self.get_logger().warning("bridge parameter services unavailable: teach damping not set")
            return
        req = GetParameters.Request(names=["damping_kd"])

        def got(future):
            res = future.result()
            if res is None or not res.values or res.values[0].type != ParameterType.PARAMETER_DOUBLE:
                return
            if self.state == RECORDING and self.saved_damping_kd is None:
                self.saved_damping_kd = res.values[0].double_value
                self._set_bridge_damping(float(self._p("teach_damping_kd")))
        self.bridge_get.call_async(req).add_done_callback(got)

    def _restore_damping(self):
        if self.saved_damping_kd is not None:
            self._set_bridge_damping(self.saved_damping_kd)
            self.saved_damping_kd = None

    def _set_bridge_damping(self, kd):
        req = SetParameters.Request(
            parameters=[Parameter("damping_kd", Parameter.Type.DOUBLE, float(kd)).to_parameter_msg()])
        self.bridge_set.call_async(req)

    # -------------------------------------------------------------- playback

    def _load_selected(self):
        name = self._p("selected_recording")
        if not name:
            raise RuntimeError("no recording selected")
        if self.joint_names is None or not self._data_fresh():
            raise RuntimeError("no fresh /joint_states")
        rec = storage.load_recording(os.path.join(self.recordings_dir, name))
        # Played joints = selected groups that are in the take; others hold.
        mask = self._group_mask(self._p("play_groups"))
        mask &= np.array([n in rec["joint_names"] for n in self.joint_names])
        played = [n for n, m in zip(self.joint_names, mask) if m]
        if not played:
            raise RuntimeError("the selected groups are not in this recording")
        offline_now = sorted(set(played) & self.offline)
        if offline_now:
            raise RuntimeError("motors offline now: " + ", ".join(offline_now))
        validate_recording(rec, played, float(self._p("max_recorded_vel")))
        rec_q = np.zeros((len(rec["t"]), len(self.joint_names)))
        rec_q[:, mask] = reorder(rec, played)
        self.rec = rec
        self.rec_q = rec_q  # only the masked columns are used
        self.rec_t = np.asarray(rec["t"], float) - float(rec["t"][0])
        self.selected_mask = mask

    def _start_pose(self):
        """Target with selected joints at frame 0 and the others where they are now."""
        base = self.target if self.target is not None else self.q
        return np.where(self.selected_mask, self.rec_q[0], base)

    def _home_pose(self, start):
        """Target with played joints at home (start pose where home has no value), or None."""
        if not self.home:
            return None
        joints = self.home["joints"]
        home = np.array([joints.get(n, np.nan) for n in self.joint_names])
        return np.where(self.selected_mask & ~np.isnan(home), home, start)

    def _near(self, pose):
        current = self.target if self.target is not None else self.q
        return float(np.max(np.abs((pose - current)[self.selected_mask]))) < AT_HOME_TOLERANCE

    def _begin_segment(self, q_to, state):
        q_from = self.target if self.target is not None else self.q.copy()
        duration = safe_move_duration(q_from, q_to, float(self._p("approach_max_vel")),
                                      float(self._p("approach_min_duration_s")))
        self.segment = (q_from, q_to, time.monotonic(), duration)
        self.state = state
        what = {GOING_HOME: "moving to home", APPROACHING: "moving to start pose",
                RETURNING: "returning to home" if self.home else "returning to start pose"}[state]
        self.message = "%s (%.1f s)" % (what, duration)

    def _command_step(self):
        if self.state not in COMMANDING:
            return
        if not self._data_fresh():
            self._trigger_estop("/joint_states timed out during %s" % self.state)
            return
        now = time.monotonic()
        if self.state in SEGMENT_STATES:
            q_from, q_to, t0, duration = self.segment
            s = (now - t0) / duration
            self.target = smooth_interp(q_from, q_to, s)
            if s >= 1.0:
                if self.pending:
                    self._begin_segment(*self.pending.pop(0))
                elif self.state == APPROACHING:
                    self.state = PLAYING
                    self.play_last = now
                    self.message = "playing"
                else:
                    self.state = HOLDING
                    self.segment = None
                    self.message = "holding home" if self.home else "holding start pose"
        elif self.state == PLAYING:
            self.play_clock += now - self.play_last
            self.play_last = now
            frame = interp_frames(self.rec_t, self.rec_q, self.play_clock)
            self.target = np.where(self.selected_mask, frame, self.target)
            if self.play_clock >= self.rec_t[-1]:
                start = self._start_pose()
                home = self._home_pose(start)
                self._begin_segment(home if home is not None else start, RETURNING)
        # PAUSED / HOLDING keep publishing the last target.
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = [float(v) for v in self.target]
        self.command_pub.publish(msg)

    # ---------------------------------------------------------------- status

    def _publish_status(self):
        now = time.monotonic()
        if now - self._listing_time > 1.0:
            self.recordings = self.index.list()
            self._listing_time = now
        status = {
            "state": self.state,
            "message": self.message,
            "data_fresh": self._data_fresh(),
            "joints": len(self.joint_names or []),
            "groups": list(self.groups),
            "play_groups": list(self._p("play_groups")),
            "record_groups": list(self._p("record_groups")),
            "offline_joints": sorted(self.offline),
            "offline_groups": self._offline_groups(),
            "selected_recording": self._p("selected_recording"),
            "recording_name": self._p("recording_name"),
            "teach_damping_kd": float(self._p("teach_damping_kd")),
            "last_saved": os.path.basename(self.last_saved) if self.last_saved else "",
            "recordings": self.recordings,
            "home": ({"created": self.home.get("created", ""), "joints": len(self.home["joints"])}
                     if self.home else None),
        }
        if self.state == RECORDING and self.buffer is not None:
            status["recording"] = {"frames": len(self.buffer["t"]),
                                   "duration_s": round(now - self.buffer["t0"], 2)}
        if self.rec is not None and self.state in (PLAYING, PAUSED):
            status["playback"] = {"t": round(self.play_clock, 2),
                                  "duration_s": round(float(self.rec_t[-1]), 2)}
        if self.segment is not None:
            status["segment"] = {"t": round(now - self.segment[2], 2),
                                 "duration_s": round(self.segment[3], 2)}
        self.status_pub.publish(String(data=json.dumps(status)))


def _raise_keyboard_interrupt(_signum, _frame):
    raise KeyboardInterrupt


def main(args=None):
    # Explicit handlers: a process started in the background from a
    # non-interactive shell inherits SIGINT as *ignored*, and Foxy's rclpy does
    # not install its own handler, so the node would not stop. The 100 Hz
    # command timer wakes the executor often enough for the handler to run.
    signal.signal(signal.SIGINT, _raise_keyboard_interrupt)
    signal.signal(signal.SIGTERM, _raise_keyboard_interrupt)
    rclpy.init(args=args)
    node = MotionRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

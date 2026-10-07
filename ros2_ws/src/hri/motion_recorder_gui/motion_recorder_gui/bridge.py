import json
import time

from rcl_interfaces.srv import SetParametersAtomically
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String
from std_srvs.srv import Trigger

SERVICE_WAIT_SEC = 1.0
SERVICE_TIMEOUT_SEC = 5.0
STATUS_STALE_SEC = 0.5
RECORDER_ACTIONS = ("record_start", "record_stop", "discard", "play", "pause", "resume",
                    "reset", "release", "set_home")


class MotionRecorderGuiBridge(Node):
    """ROS side of the motion recorder GUI (served by getup_gui.server.run_server)."""

    def __init__(self):
        super().__init__("motion_recorder_gui_bridge")
        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("port", 8083)
        self.declare_parameter("recorder_node", "/motion_recorder")
        self.declare_parameter("bridge_node", "/g1_lowlevel_bridge")

        self.host = self.get_parameter("host").value
        self.port = self.get_parameter("port").value
        recorder = self.get_parameter("recorder_node").value
        bridge = self.get_parameter("bridge_node").value
        self._names = {"recorder": recorder, "bridge": bridge}

        self._triggers = {a: self.create_client(Trigger, "%s/%s" % (recorder, a))
                          for a in RECORDER_ACTIONS}
        self._set_clients = {
            "recorder": self.create_client(SetParametersAtomically,
                                           recorder + "/set_parameters_atomically"),
            "bridge": self.create_client(SetParametersAtomically,
                                         bridge + "/set_parameters_atomically"),
        }

        self._status = {"recorder": (None, 0.0), "bridge": (None, 0.0)}
        self.create_subscription(String, recorder + "/status",
                                 lambda m: self._on_status("recorder", m), 10)
        self.create_subscription(String, bridge + "/status",
                                 lambda m: self._on_status("bridge", m), 10)

    def config(self):
        return {"recorder_node": self._names["recorder"], "bridge_node": self._names["bridge"]}

    def status_snapshot(self):
        now = time.monotonic()
        out = {}
        for key, (status, stamp) in self._status.items():
            online = status is not None and now - stamp < STATUS_STALE_SEC
            out[key] = dict(status or {}, online=online)
        return out

    def _on_status(self, key, msg):
        try:
            self._status[key] = (json.loads(msg.data), time.monotonic())
        except ValueError:
            self.get_logger().warning("invalid %s status" % key)

    # -- blocking commands (worker threads) ------------------------------------

    def execute_command(self, command):
        action = command.get("action")
        if action == "record_start":
            params = {
                "recording_name": ("string", command.get("name") or ""),
                "teach_damping_kd": ("double", command.get("teach_damping_kd", 0.3)),
            }
            if command.get("groups"):
                params["record_groups"] = ("string_array", command["groups"])
            self._set("recorder", params)
            return self._trigger("record_start")
        if action in ("play", "reset"):
            params = {"selected_recording": ("string", command.get("file") or "")}
            if command.get("groups"):
                params["play_groups"] = ("string_array", command["groups"])
            self._set("recorder", params)
            return self._trigger(action)
        if action in ("record_stop", "discard", "pause", "resume", "release", "set_home"):
            return self._trigger(action)
        if action == "select":
            self._set("recorder", {"selected_recording": ("string", command.get("file") or "")})
            return {"ok": True}
        if action == "set_teach_damping":
            self._set("recorder", {"teach_damping_kd": ("double", command.get("value"))})
            return {"ok": True}
        if action == "set_enable_lowcmd":
            self._set("bridge", {"enable_lowcmd": ("bool", bool(command.get("enabled")))})
            return {"ok": True}
        raise ValueError("unknown_action:%s" % action)

    def _wait_and_call(self, client, request):
        if not client.wait_for_service(timeout_sec=SERVICE_WAIT_SEC):
            raise RuntimeError("service_unavailable:%s" % client.srv_name)
        future = client.call_async(request)
        deadline = time.monotonic() + SERVICE_TIMEOUT_SEC
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.005)
        if not future.done() or future.result() is None:
            raise RuntimeError("service_failed:%s" % client.srv_name)
        return future.result()

    def _trigger(self, action):
        result = self._wait_and_call(self._triggers[action], Trigger.Request())
        return {"ok": result.success, "message": result.message}

    def _set(self, node, values):
        types = {"string": (Parameter.Type.STRING, str), "double": (Parameter.Type.DOUBLE, float),
                 "bool": (Parameter.Type.BOOL, bool),
                 "string_array": (Parameter.Type.STRING_ARRAY, lambda v: [str(x) for x in v])}
        request = SetParametersAtomically.Request()
        try:
            request.parameters = [
                Parameter(name, types[t][0], types[t][1](value)).to_parameter_msg()
                for name, (t, value) in values.items()]
        except (TypeError, ValueError) as exc:
            raise ValueError("invalid_value:%s" % exc)
        result = self._wait_and_call(self._set_clients[node], request).result
        if not result.successful:
            raise RuntimeError(result.reason)

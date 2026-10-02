import json
import time

from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters, SetParametersAtomically
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Empty, String
from std_srvs.srv import Trigger


# Scalar hyperparameters of getup_policy_node (getup/config/g1_getup_policy.yaml).
POLICY_SPECS = [
    {"name": "action_clip", "label": "Action clip (0 = off)", "unit": "", "type": "double",
     "min": 0.0, "max": 100.0, "step": 0.1},
    {"name": "settle_steps", "label": "Settle steps after start", "unit": "steps", "type": "int",
     "min": 0, "max": 500, "step": 1},
    {"name": "policy_rate_hz", "label": "Policy rate", "unit": "Hz", "type": "double",
     "min": 1.0, "max": 1000.0, "step": 1.0},
    {"name": "command_rate_hz", "label": "Command rate", "unit": "Hz", "type": "double",
     "min": 1.0, "max": 2000.0, "step": 1.0},
    {"name": "data_timeout_s", "label": "Sensor data timeout", "unit": "s", "type": "double",
     "min": 0.01, "max": 2.0, "step": 0.01},
    {"name": "auto_start", "label": "Auto start when data arrives", "unit": "", "type": "bool"},
]

# Scalar hyperparameters of g1_lowlevel_bridge (getup/config/g1_lowlevel_bridge.yaml).
BRIDGE_SPECS = [
    {"name": "damping_kd", "label": "Damping Kd (e-stop / no command)", "unit": "N·m·s/rad",
     "type": "double", "min": 0.0, "max": 20.0, "step": 0.1},
    {"name": "max_target_delta", "label": "Max |target - q| (0 = off)", "unit": "rad",
     "type": "double", "min": 0.0, "max": 6.0, "step": 0.01},
    {"name": "command_timeout_s", "label": "Command timeout", "unit": "s", "type": "double",
     "min": 0.01, "max": 2.0, "step": 0.01},
    {"name": "heartbeat_timeout_s", "label": "Dead-man timeout (0 = off)", "unit": "s",
     "type": "double", "min": 0.0, "max": 5.0, "step": 0.05},
]

# Per-joint arrays shown as columns of the joint table: (node, parameter).
JOINT_COLUMNS = [
    {"node": "policy", "name": "default_joint_pos", "label": "Default pos", "unit": "rad",
     "step": 0.001},
    {"node": "policy", "name": "action_scale", "label": "Action scale", "unit": "",
     "step": 0.01},
    {"node": "bridge", "name": "kp", "label": "Kp", "unit": "N·m/rad", "step": 0.1},
    {"node": "bridge", "name": "kd", "label": "Kd", "unit": "N·m·s/rad", "step": 0.01},
    {"node": "bridge", "name": "position_lower", "label": "Lower limit", "unit": "rad",
     "step": 0.01},
    {"node": "bridge", "name": "position_upper", "label": "Upper limit", "unit": "rad",
     "step": 0.01},
]

# Read-only values the page needs (labels, current model, motor output).
POLICY_EXTRA = ["joint_names", "observation_terms", "observation_scales", "policy_path"]
BRIDGE_EXTRA = ["joint_names", "enable_lowcmd"]

TYPE_MAP = {
    "double": (Parameter.Type.DOUBLE, float),
    "int": (Parameter.Type.INTEGER, int),
    "bool": (Parameter.Type.BOOL, bool),
    "string": (Parameter.Type.STRING, str),
    "double_array": (Parameter.Type.DOUBLE_ARRAY, lambda v: [float(x) for x in v]),
}


def _param_types():
    types = {"policy": {}, "bridge": {}}
    for spec in POLICY_SPECS:
        types["policy"][spec["name"]] = spec["type"]
    for spec in BRIDGE_SPECS:
        types["bridge"][spec["name"]] = spec["type"]
    for col in JOINT_COLUMNS:
        types[col["node"]][col["name"]] = "double_array"
    types["policy"]["observation_scales"] = "double_array"
    types["policy"]["policy_path"] = "string"
    types["bridge"]["enable_lowcmd"] = "bool"
    return types


PARAM_TYPES = _param_types()

SERVICE_WAIT_SEC = 1.0
SERVICE_TIMEOUT_SEC = 5.0
# A status message older than this means the node is not running.
STATUS_STALE_SEC = 0.5


class GetupGuiBridge(Node):
    def __init__(self):
        super().__init__("getup_gui_bridge")
        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("port", 8082)
        self.declare_parameter("policy_node", "/getup_policy_node")
        self.declare_parameter("bridge_node", "/g1_lowlevel_bridge")
        self.declare_parameter("estop_topic", "/getup/estop")
        self.declare_parameter("heartbeat_topic", "/getup/heartbeat")

        self.host = self.get_parameter("host").value
        self.port = self.get_parameter("port").value
        self._targets = {
            "policy": self.get_parameter("policy_node").value,
            "bridge": self.get_parameter("bridge_node").value,
        }

        self._get_clients = {
            key: self.create_client(GetParameters, f"{target}/get_parameters")
            for key, target in self._targets.items()
        }
        self._set_clients = {
            key: self.create_client(SetParametersAtomically, f"{target}/set_parameters_atomically")
            for key, target in self._targets.items()
        }
        policy, bridge = self._targets["policy"], self._targets["bridge"]
        self._trigger_clients = {
            "policy_start": self.create_client(Trigger, f"{policy}/start"),
            "policy_stop": self.create_client(Trigger, f"{policy}/stop"),
            "bridge_estop": self.create_client(Trigger, f"{bridge}/estop"),
            "bridge_reset": self.create_client(Trigger, f"{bridge}/reset_estop"),
        }

        reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self._estop_pub = self.create_publisher(
            Bool, self.get_parameter("estop_topic").value, reliable)
        self._heartbeat_pub = self.create_publisher(
            Empty, self.get_parameter("heartbeat_topic").value, reliable)

        self._status = {"policy": (None, 0.0), "bridge": (None, 0.0)}
        self.create_subscription(
            String, f"{policy}/status", lambda m: self._on_status("policy", m), 10)
        self.create_subscription(
            String, f"{bridge}/status", lambda m: self._on_status("bridge", m), 10)

    def config(self):
        return {
            "policy_node": self._targets["policy"],
            "bridge_node": self._targets["bridge"],
            "policy_specs": POLICY_SPECS,
            "bridge_specs": BRIDGE_SPECS,
            "joint_columns": JOINT_COLUMNS,
        }

    # -- fast paths, called directly on the asyncio thread ---------------------

    def heartbeat(self):
        """Forwards one browser heartbeat to the bridge's dead-man."""
        self._heartbeat_pub.publish(Empty())

    def publish_estop(self):
        """Fire-and-forget e-stop: both getup nodes latch/stop on this topic."""
        self._estop_pub.publish(Bool(data=True))

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
            self.get_logger().warning(f"invalid {key} status: {msg.data}")

    # -- blocking commands, run on worker threads (see server.py) --------------

    def execute_command(self, command):
        action = command.get("action")
        if action == "read_parameters":
            return self._read_all()
        if action == "set_parameters":
            return self._set_target_parameters(
                command.get("node"), command.get("parameters") or {})
        if action == "estop_services":
            return self._estop_services()
        if action in ("policy_start", "policy_stop"):
            return self._trigger(action)
        if action == "reset_estop":
            return self._trigger("bridge_reset")
        raise ValueError(f"unknown_action:{action}")

    def _wait_and_call(self, client, request, timeout_sec=SERVICE_TIMEOUT_SEC):
        if not client.wait_for_service(timeout_sec=SERVICE_WAIT_SEC):
            raise RuntimeError(f"service_unavailable:{client.srv_name}")
        future = client.call_async(request)
        deadline = time.monotonic() + timeout_sec
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.005)
        if not future.done():
            raise RuntimeError(f"service_timeout:{client.srv_name}")
        result = future.result()
        if result is None:
            raise RuntimeError(f"service_failed:{client.srv_name}")
        return result

    def _trigger(self, name):
        result = self._wait_and_call(self._trigger_clients[name], Trigger.Request())
        return {"ok": result.success, "message": result.message}

    def _estop_services(self):
        # Belt and braces after the topic: confirm both nodes reacted. Sent
        # without waiting for each other so one slow node cannot delay the other.
        futures = {}
        for name in ("bridge_estop", "policy_stop"):
            client = self._trigger_clients[name]
            if client.service_is_ready():
                futures[name] = client.call_async(Trigger.Request())
        deadline = time.monotonic() + SERVICE_TIMEOUT_SEC
        while any(not f.done() for f in futures.values()) and time.monotonic() < deadline:
            time.sleep(0.002)
        results = {
            name: bool(f.done() and f.result() is not None and f.result().success)
            for name, f in futures.items()
        }
        for name in ("bridge_estop", "policy_stop"):
            results.setdefault(name, False)
        return {"ok": results["bridge_estop"], "results": results}

    def _read_node(self, key, names):
        request = GetParameters.Request()
        request.names = names
        result = self._wait_and_call(self._get_clients[key], request)
        return {name: self._decode_value(value) for name, value in zip(names, result.values)}

    def _read_all(self):
        policy_names = [s["name"] for s in POLICY_SPECS] + POLICY_EXTRA + [
            c["name"] for c in JOINT_COLUMNS if c["node"] == "policy"]
        bridge_names = [s["name"] for s in BRIDGE_SPECS] + BRIDGE_EXTRA + [
            c["name"] for c in JOINT_COLUMNS if c["node"] == "bridge"]
        values, errors = {}, {}
        for key, names in (("policy", policy_names), ("bridge", bridge_names)):
            try:
                values[key] = self._read_node(key, names)
            except RuntimeError as exc:
                errors[key] = str(exc)
        return {"ok": bool(values), "parameters": values, "errors": errors}

    @staticmethod
    def _decode_value(value):
        t = value.type
        if t == ParameterType.PARAMETER_DOUBLE:
            return value.double_value
        if t == ParameterType.PARAMETER_INTEGER:
            return value.integer_value
        if t == ParameterType.PARAMETER_BOOL:
            return value.bool_value
        if t == ParameterType.PARAMETER_STRING:
            return value.string_value
        if t == ParameterType.PARAMETER_DOUBLE_ARRAY:
            return list(value.double_array_value)
        if t == ParameterType.PARAMETER_INTEGER_ARRAY:
            return list(value.integer_array_value)
        if t == ParameterType.PARAMETER_BOOL_ARRAY:
            return list(value.bool_array_value)
        if t == ParameterType.PARAMETER_STRING_ARRAY:
            return list(value.string_array_value)
        return None

    def _set_target_parameters(self, key, values):
        # Not named _set_parameters to avoid shadowing rclpy.node.Node's own
        # internal method (see walk_gui/bridge.py).
        if key not in self._targets:
            raise ValueError(f"unknown_node:{key}")
        types = PARAM_TYPES[key]
        unknown = [name for name in values if name not in types]
        if unknown:
            raise ValueError(f"unknown_parameters:{','.join(unknown)}")
        request = SetParametersAtomically.Request()
        try:
            request.parameters = [
                Parameter(name, TYPE_MAP[types[name]][0], TYPE_MAP[types[name]][1](value))
                .to_parameter_msg()
                for name, value in values.items()
            ]
        except (TypeError, ValueError) as exc:
            raise ValueError(f"invalid_value:{exc}")
        result = self._wait_and_call(self._set_clients[key], request).result
        return {"ok": result.successful, "node": key, "reason": result.reason}

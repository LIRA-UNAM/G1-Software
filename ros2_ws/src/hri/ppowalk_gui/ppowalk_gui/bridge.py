import json
import time

from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters, SetParametersAtomically
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String
from std_srvs.srv import Trigger


def _num(name, label, unit, lo, hi, step):
    return {"name": name, "label": label, "unit": unit, "type": "double",
            "min": lo, "max": hi, "step": step}


# Scalar hyperparameters of ppowalk_policy_node (ppowalk/config/g1_ppowalk_policy.yaml).
POLICY_SPECS = [
    _num("max_start_tilt_deg", "Max tilt to allow Start", "deg", 1.0, 90.0, 1.0),
    _num("start_ramp_s", "Start ramp (blend into policy)", "s", 0.0, 10.0, 0.1),
    _num("switch_max_ang_vel", "Policy switch: max body angular velocity", "rad/s", 0.01, 5.0, 0.05),
    _num("switch_max_joint_vel", "Policy switch: max leg joint velocity", "rad/s", 0.01, 10.0, 0.05),
    _num("switch_settle_s", "Policy switch: time at rest", "s", 0.0, 5.0, 0.1),
    _num("home_max_vel", "Home: peak joint speed", "rad/s", 0.05, 2.0, 0.05),
    _num("home_min_duration_s", "Home: minimum duration", "s", 0.0, 20.0, 0.5),
    _num("action_clip", "Action clip (0 = off)", "", 0.0, 100.0, 0.1),
    _num("policy_rate_hz", "Policy rate", "Hz", 1.0, 1000.0, 1.0),
    _num("command_rate_hz", "Command rate", "Hz", 1.0, 2000.0, 1.0),
    _num("data_timeout_s", "Sensor data timeout", "s", 0.01, 2.0, 0.01),
    _num("lin_vel_x_min", "vx min", "m/s", -3.0, 0.0, 0.05),
    _num("lin_vel_x_max", "vx max", "m/s", 0.0, 3.0, 0.05),
    _num("lin_vel_y_min", "vy min", "m/s", -3.0, 0.0, 0.05),
    _num("lin_vel_y_max", "vy max", "m/s", 0.0, 3.0, 0.05),
    _num("ang_vel_z_min", "wz min", "rad/s", -3.0, 0.0, 0.05),
    _num("ang_vel_z_max", "wz max", "rad/s", 0.0, 3.0, 0.05),
    {"name": "detect_offline", "label": "Pin obs of motors detected offline", "unit": "",
     "type": "bool"},
    _num("cmd_vel_timeout_s", "cmd_vel timeout (falls back to 0)", "s", 0.05, 5.0, 0.05),
]

# Velocity-command calibration (shown in the command panel).
OFFSET_SPECS = [
    _num("cmd_vx_offset", "vx offset", "m/s", -0.5, 0.5, 0.01),
    _num("cmd_vy_offset", "vy offset", "m/s", -0.5, 0.5, 0.01),
    _num("cmd_wz_offset", "wz offset", "rad/s", -0.5, 0.5, 0.01),
    {"name": "offsets_when_standing", "label": "Apply offsets with a zero command", "unit": "",
     "type": "bool"},
]

# Wireless remote (shown in the command panel).
REMOTE_SPECS = [
    {"name": "remote_start", "label": "Remote Start button starts the policy", "unit": "",
     "type": "bool"},
    _num("remote_deadzone", "Stick deadzone", "", 0.0, 0.9, 0.01),
    _num("remote_timeout_s", "Remote data timeout (falls back to 0)", "s", 0.05, 5.0, 0.05),
]

# Velocity command source -> (use_remote, use_cmd_vel); set atomically.
SOURCE_PARAMS = ["use_remote", "use_cmd_vel"]
COMMAND_SOURCES = {"remote": (True, False), "gui": (False, False), "cmd_vel": (False, True)}

# Scalar parameters of getup's g1_lowlevel_bridge.
BRIDGE_SPECS = [
    _num("damping_kd", "Damping Kd (no command / Release)", "N·m·s/rad", 0.0, 20.0, 0.1),
    _num("max_target_delta", "Max |target - q| (0 = off)", "rad", 0.0, 6.0, 0.01),
    _num("command_timeout_s", "Command timeout", "s", 0.01, 2.0, 0.01),
]

# Per-joint arrays of the joint table. "index_by" names the parameter whose
# joint names index the array (action_scale only covers the leg joints).
JOINT_COLUMNS = [
    {"node": "policy", "name": "default_joint_pos", "label": "Default pos", "unit": "rad",
     "step": 0.001, "index_by": "joint_names"},
    {"node": "policy", "name": "home_pose", "label": "Home pose", "unit": "rad",
     "step": 0.01, "index_by": "joint_names"},
    {"node": "policy", "name": "held_joint_targets", "label": "Hold target", "unit": "rad",
     "step": 0.01, "index_by": "joint_names"},
    {"node": "policy", "name": "action_scale", "label": "Action scale", "unit": "",
     "step": 0.01, "index_by": "action_joint_names"},
    {"node": "bridge", "name": "kp", "label": "Kp", "unit": "N·m/rad", "step": 0.1,
     "index_by": "joint_names"},
    {"node": "bridge", "name": "kd", "label": "Kd", "unit": "N·m·s/rad", "step": 0.01,
     "index_by": "joint_names"},
    {"node": "bridge", "name": "position_lower", "label": "Lower limit", "unit": "rad",
     "step": 0.01, "index_by": "joint_names"},
    {"node": "bridge", "name": "position_upper", "label": "Upper limit", "unit": "rad",
     "step": 0.01, "index_by": "joint_names"},
]

COMMAND_PARAMS = ["cmd_vx", "cmd_vy", "cmd_wz"]
# Joint-name lists shown as checkbox columns of the joint table.
JOINT_FLAGS = [
    {"name": "command_joints", "label": "Commanded",
     "title": "Published to the bridge (others are damped). Only while the policy is stopped."},
    {"name": "obs_pinned_joints", "label": "Pin obs",
     "title": "Report this joint at its hold target, at rest, in the observation."},
]
POLICY_EXTRA = ["joint_names", "action_joint_names", "observation_terms", "observation_scales",
                "policy_path", "always_commanded_joints"] + COMMAND_PARAMS + SOURCE_PARAMS + [
                    f["name"] for f in JOINT_FLAGS]
BRIDGE_EXTRA = ["joint_names", "enable_lowcmd"]

TYPE_MAP = {
    "double": (Parameter.Type.DOUBLE, float),
    "bool": (Parameter.Type.BOOL, bool),
    "string": (Parameter.Type.STRING, str),
    "double_array": (Parameter.Type.DOUBLE_ARRAY, lambda v: [float(x) for x in v]),
    "string_array": (Parameter.Type.STRING_ARRAY, lambda v: [str(x) for x in v]),
}


def _param_types():
    types = {"policy": {}, "bridge": {}}
    for spec in POLICY_SPECS + OFFSET_SPECS + REMOTE_SPECS:
        types["policy"][spec["name"]] = spec["type"]
    for name in SOURCE_PARAMS:
        types["policy"][name] = "bool"
    for spec in BRIDGE_SPECS:
        types["bridge"][spec["name"]] = spec["type"]
    for col in JOINT_COLUMNS:
        types[col["node"]][col["name"]] = "double_array"
    for name in COMMAND_PARAMS:
        types["policy"][name] = "double"
    for flag in JOINT_FLAGS:
        types["policy"][flag["name"]] = "string_array"
    types["policy"]["observation_scales"] = "double_array"
    types["policy"]["policy_path"] = "string"
    types["bridge"]["enable_lowcmd"] = "bool"
    return types


PARAM_TYPES = _param_types()
TRIGGERS = ("start", "stop", "release", "home")
SERVICE_WAIT_SEC = 1.0
SERVICE_TIMEOUT_SEC = 5.0
STATUS_STALE_SEC = 0.5


class PpowalkGuiBridge(Node):
    """ROS side of the ppowalk GUI (served by getup_gui.server.run_server)."""

    def __init__(self):
        super().__init__("ppowalk_gui_bridge")
        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("port", 8084)
        self.declare_parameter("policy_node", "/ppowalk_policy_node")
        self.declare_parameter("bridge_node", "/g1_lowlevel_bridge")
        self.host = self.get_parameter("host").value
        self.port = self.get_parameter("port").value
        self._targets = {"policy": self.get_parameter("policy_node").value,
                         "bridge": self.get_parameter("bridge_node").value}
        self._get_clients = {k: self.create_client(GetParameters, t + "/get_parameters")
                             for k, t in self._targets.items()}
        self._set_clients = {
            k: self.create_client(SetParametersAtomically, t + "/set_parameters_atomically")
            for k, t in self._targets.items()}
        self._triggers = {a: self.create_client(Trigger, "%s/%s" % (self._targets["policy"], a))
                          for a in TRIGGERS}
        self._status = {"policy": (None, 0.0), "bridge": (None, 0.0)}
        for key in self._targets:
            self.create_subscription(String, self._targets[key] + "/status",
                                     lambda m, k=key: self._on_status(k, m), 10)

    def config(self):
        return {"policy_node": self._targets["policy"], "bridge_node": self._targets["bridge"],
                "policy_specs": POLICY_SPECS, "offset_specs": OFFSET_SPECS,
                "remote_specs": REMOTE_SPECS,
                "bridge_specs": BRIDGE_SPECS,
                "joint_columns": JOINT_COLUMNS, "joint_flags": JOINT_FLAGS}

    def status_snapshot(self):
        now = time.monotonic()
        return {k: dict(s or {}, online=s is not None and now - t < STATUS_STALE_SEC)
                for k, (s, t) in self._status.items()}

    def _on_status(self, key, msg):
        try:
            self._status[key] = (json.loads(msg.data), time.monotonic())
        except ValueError:
            self.get_logger().warning("invalid %s status" % key)

    # -- blocking commands (worker threads, see getup_gui.server) ---------------

    def execute_command(self, command):
        action = command.get("action")
        if action == "read_parameters":
            return self._read_all()
        if action == "set_parameters":
            return self._set_target_parameters(command.get("node"), command.get("parameters") or {})
        if action == "set_command":
            values = {name: command.get(key, 0.0)
                      for name, key in zip(COMMAND_PARAMS, ("vx", "vy", "wz"))}
            return self._set_target_parameters("policy", values)
        if action == "set_source":
            source = command.get("source")
            if source not in COMMAND_SOURCES:
                raise ValueError("unknown_source:%s" % source)
            return self._set_target_parameters(
                "policy", dict(zip(SOURCE_PARAMS, COMMAND_SOURCES[source])))
        if action in TRIGGERS:
            result = self._wait_and_call(self._triggers[action], Trigger.Request())
            return {"ok": result.success, "message": result.message}
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

    def _read_all(self):
        names = {
            "policy": [s["name"] for s in POLICY_SPECS + OFFSET_SPECS + REMOTE_SPECS] +
            POLICY_EXTRA + [
                c["name"] for c in JOINT_COLUMNS if c["node"] == "policy"],
            "bridge": [s["name"] for s in BRIDGE_SPECS] + BRIDGE_EXTRA + [
                c["name"] for c in JOINT_COLUMNS if c["node"] == "bridge"],
        }
        values, errors = {}, {}
        for key, keys in names.items():
            try:
                request = GetParameters.Request(names=keys)
                result = self._wait_and_call(self._get_clients[key], request)
                values[key] = {n: self._decode(v) for n, v in zip(keys, result.values)}
            except RuntimeError as exc:
                errors[key] = str(exc)
        return {"ok": bool(values), "parameters": values, "errors": errors}

    @staticmethod
    def _decode(value):
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
        if t == ParameterType.PARAMETER_STRING_ARRAY:
            return list(value.string_array_value)
        return None

    def _set_target_parameters(self, key, values):
        if key not in self._targets:
            raise ValueError("unknown_node:%s" % key)
        types = PARAM_TYPES[key]
        unknown = [n for n in values if n not in types]
        if unknown:
            raise ValueError("unknown_parameters:%s" % ",".join(unknown))
        request = SetParametersAtomically.Request()
        try:
            request.parameters = [
                Parameter(n, TYPE_MAP[types[n]][0], TYPE_MAP[types[n]][1](v)).to_parameter_msg()
                for n, v in values.items()]
        except (TypeError, ValueError) as exc:
            raise ValueError("invalid_value:%s" % exc)
        result = self._wait_and_call(self._set_clients[key], request).result
        return {"ok": result.successful, "node": key, "reason": result.reason}

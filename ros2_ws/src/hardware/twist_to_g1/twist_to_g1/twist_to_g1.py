import json, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from builtin_interfaces.msg import Time
from unitree_api.msg import Request as UnitreeRequest

def now_stamp(node: Node) -> Time:
    return node.get_clock().now().to_msg()

class CmdVelBridge(Node):
    def __init__(self):
        super().__init__('twist_to_g1_bridge')
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('command_duration_s', 0.20)
        self.declare_parameter('min_linear_speed', 0.20)
        self.declare_parameter('max_linear_speed', 0.80)
        self.declare_parameter('max_lateral_speed', 0.40)
        self.declare_parameter('max_yaw_rate', 0.80)
        self.declare_parameter('deadman_timeout_s', 0.50)
        self.declare_parameter('priority', 1)

        gp = self.get_parameter
        self.pub_rate_hz = float(gp('publish_rate_hz').value)
        self.cmd_duration = float(gp('command_duration_s').value)
        self.min_lin = float(gp('min_linear_speed').value)
        self.max_lin = float(gp('max_linear_speed').value)
        self.max_lat = float(gp('max_lateral_speed').value)
        self.max_yaw = float(gp('max_yaw_rate').value)
        self.deadman = float(gp('deadman_timeout_s').value)
        self.priority = int(gp('priority').value)

        self.sub = self.create_subscription(Twist, '/cmd_vel', self._on_twist, 10)
        self.pub = self.create_publisher(UnitreeRequest, '/api/loco/request', 10)

        self.last_twist = Twist()
        self.last_cmd_time = time.monotonic()
        self.seq_id = 100

        self.create_timer(1.0/self.pub_rate_hz, self._tick)
        self.get_logger().info('twist_to_g1: /cmd_vel -> /api/loco/request.')

    def _on_twist(self, msg: Twist):
        self.last_twist = msg
        self.last_cmd_time = time.monotonic()

    def _sat(self, v, lim):
        return max(min(v, lim), -lim)

    def _build_req(self, vx, vy, wz) -> UnitreeRequest:
        m = UnitreeRequest()
        m.header.identity.id = self.seq_id; self.seq_id += 1
        m.header.identity.api_id = 7105
        m.header.lease.id = 0
        m.header.policy.priority = self.priority
        m.header.policy.noreply = False

        m.parameter = json.dumps({"velocity": [vx, vy, wz], "duration": self.cmd_duration})
        m.binary = []
        return m

    def _tick(self):
        vx = self._sat(self.last_twist.linear.x, self.max_lin)
        vy = self._sat(self.last_twist.linear.y, self.max_lat)
        wz = self._sat(self.last_twist.angular.z, self.max_yaw)

        if abs(vx) < self.min_lin: vx = 0.0
        if abs(vy) < self.min_lin: vy = 0.0

        if time.monotonic() - self.last_cmd_time > self.deadman:
            vx = vy = wz = 0.0

        self.pub.publish(self._build_req(vx, vy, wz))

def main():
    rclpy.init()
    node = CmdVelBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

import json, time
import rclpy
from rclpy.node import Node
from unitree_api.msg import Request as UnitreeRequest

class G1Initializer(Node):
    def __init__(self):
        super().__init__('g1_initializer')
        self.pub = self.create_publisher(UnitreeRequest, '/api/loco/request', 10)
        self.seq = 500
        self.declare_parameter('priority', 5)
        self.priority = int(self.get_parameter('priority').value)
        self.get_logger().info('Init: damping -> stand-up -> balance -> start')
        self._oneshot_timer = self.create_timer(0.1, self._run_once)

    def _msg(self, api_id, d, prio=None):
        if prio is None:
            prio = self.priority
        m = UnitreeRequest()
        m.header.identity.id = self.seq; self.seq += 1
        m.header.identity.api_id = api_id
        m.header.lease.id = 0
        m.header.policy.priority = prio
        m.header.policy.noreply = False
        m.parameter = json.dumps(d)
        m.binary = []
        return m

    def _run_once(self):
        try:
            self._oneshot_timer.cancel()
        except Exception:
            pass

        self.get_logger().info('DAMPING...')
        self.pub.publish(self._msg(7101, {"data": 1}))
        time.sleep(5.0)

        self.get_logger().info('STAND-UP...')
        self.pub.publish(self._msg(7101, {"data": 4}))
        time.sleep(10.0)

        self.get_logger().info('BALANCE ON...')
        self.pub.publish(self._msg(7102, {"data": 1}))
        time.sleep(5.0)

        self.get_logger().info('START ROUTINE...')
        self.pub.publish(self._msg(7101, {"data": 500}))
        self.get_logger().info('Listo para /cmd_vel.')

def main():
    rclpy.init()
    node = G1Initializer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

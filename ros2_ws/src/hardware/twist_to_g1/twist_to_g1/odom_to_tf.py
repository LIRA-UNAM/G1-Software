import math
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from unitree_go.msg import SportModeState

class OdomToTFNode(Node):
    def __init__(self):
        super().__init__('odom_to_tf_node')
        self.broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info("Odom to TF Node has been started.")
        self.subscription = self.create_subscription(SportModeState, '/lf/odommodestate', self.callback_odom_state, 10)
        self.robot_x=0.0
        self.robot_y=0.0
        self.robot_a=0.0

    def callback_odom_state(self, msg):
        self.get_logger().info(f"Received SportModeState:")
        print(msg.position.)
        

    def timer_callback(self):
        t = self.get_clock().now().to_msg()
        transform = TransformStamped()
        transform.header.stamp = t
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_link'
        
        # Example static transform values
        transform.transform.translation.x = 1.0
        transform.transform.translation.y = 2.0
        transform.transform.translation.z = 0.0
        
        quat = self.euler_to_quaternion(0.0, 0.0, math.radians(45))
        transform.transform.rotation.x = quat[0]
        transform.transform.rotation.y = quat[1]
        transform.transform.rotation.z = quat[2]
        transform.transform.rotation.w = quat[3]

        self.broadcaster.sendTransform(transform)

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return (qx, qy, qz, qw)


def main():
    rclpy.init()
    node = OdomToTFNode()
    rclpy.spin(node)
    node.destroy_node()

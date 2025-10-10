#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool


class EnablePublisher(Node):
    def __init__(self):
        super().__init__("enable_publisher")

        # Declare and read parameter (default = False)
        self.declare_parameter("use_namespace", False)
        use_namespace = self.get_parameter("use_namespace").get_parameter_value().bool_value

        suffix = "/navigation/potential_fields/enable"

        if use_namespace:
            prefix = self.get_namespace()
            # ensure single slash handling
            if prefix == "/":
                topic = suffix
            else:
                topic = prefix + suffix
        else:
            topic = suffix

        self.pub = self.create_publisher(Bool, topic, 10)
        msg = Bool(data=True)
        self.pub.publish(msg)

        self.get_logger().info('Published enable message')


def main(args=None):
    rclpy.init(args=args)
    node = EnablePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

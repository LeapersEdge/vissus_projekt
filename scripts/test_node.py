#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TestPythonNode(Node):
    def __init__(self):
        super().__init__('test_python_node')
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World!'
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TestPythonNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


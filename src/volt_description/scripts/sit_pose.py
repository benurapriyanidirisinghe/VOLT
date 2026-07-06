#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class VoltSitPose(Node):
    def __init__(self):
        super().__init__("volt_sit_pose")
        self.publisher = self.create_publisher(String, "/volt/action", 10)

    def run_sequence(self):
        start = time.monotonic()
        while self.publisher.get_subscription_count() == 0:
            if time.monotonic() - start > 10.0:
                self.get_logger().error("VOLT motion controller is not available.")
                return False
            rclpy.spin_once(self, timeout_sec=0.1)
        message = String()
        message.data = "sit"
        self.publisher.publish(message)
        self.get_logger().info("Sit command sent.")
        return True


def main():
    rclpy.init()
    node = VoltSitPose()
    if node.run_sequence():
        time.sleep(0.5)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

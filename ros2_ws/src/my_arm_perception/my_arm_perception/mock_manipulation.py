#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

class MockManipulation(Node):
    def __init__(self):
        super().__init__("mock_manipulation")

        self.declare_parameter("result", "SUCCEEDED")
        self.declare_parameter("execution_delay_s", 2.0)

        self.result = self.get_parameter("result").value
        self.execution_delay_s = float(
            self.get_parameter("execution_delay_s").value
        )

        self.status_pub = self.create_publisher(
            String,
            "/medshelf/manipulation_status",
            10,
        )
        self.target_sub = self.create_subscription(
            PoseStamped,
            "/medshelf/pick_target",
            self.target_callback,
            10,
        )

        self.has_received_target = False
        self.execution_timer = None

        self.publish_status("IDLE")
        self.get_logger().info(
            f"Mock manipulation ready. Result: {self.result}"
        )

    def target_callback(self, target: PoseStamped):
        if self.has_received_target:
            return

        self.has_received_target = True

        self.get_logger().info(
            "Received pick target. Simulating manipulation."
        )
        self.publish_status("EXECUTING")

        self.execution_timer = self.create_timer(
            self.execution_delay_s,
            self.finish_execution,
        )

    def finish_execution(self):
        self.execution_timer.cancel()
        self.publish_status(self.result)

        self.get_logger().info(
            f"Mock manipulation result: {self.result}"
        )

    def publish_status(self, status):
        status_msg = String()
        status_msg.data = status
        self.status_pub.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MockManipulation()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

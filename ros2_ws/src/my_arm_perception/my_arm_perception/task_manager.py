#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, String
from my_arm_perception.bin_config import SUPPLY_TO_COLOR


class TaskManager(Node):
    def __init__(self):
        super().__init__("task_manager")

        self.declare_parameter("requested_bin", "gauze")

        self.requested_bin = self.get_parameter("requested_bin").value
        self.expected_color = SUPPLY_TO_COLOR.get(self.requested_bin)

        self.state = "IDLE"
        self.latest_visible = False
        self.latest_color = "none"
        self.latest_slot = "none"
        self.pick_target_sent = False

        self.task_status_pub = self.create_publisher(
            String,
            "/medshelf/task_status",
            10,
        )
        self.pick_target_pub = self.create_publisher(
            PoseStamped,
            "/medshelf/pick_target",
            10,
        )

        self.visible_sub = self.create_subscription(
            Bool,
            "/medshelf/target_visible",
            self.visible_callback,
            10,
        )
        self.color_sub = self.create_subscription(
            String,
            "/medshelf/detected_color",
            self.color_callback,
            10,
        )
        self.slot_sub = self.create_subscription(
            String,
            "/medshelf/detected_slot",
            self.slot_callback,
            10,
        )
        self.target_sub = self.create_subscription(
            PoseStamped,
            "/medshelf/detected_target",
            self.target_callback,
            10,
        )
        self.manipulation_sub = self.create_subscription(
            String,
            "/medshelf/manipulation_status",
            self.manipulation_callback,
            10,
        )

        self.validate_request()

    def validate_request(self):
        if self.expected_color is None:
            self.set_state("FAILED_INVALID_REQUEST")
            self.get_logger().error(
                f"Invalid requested_bin '{self.requested_bin}'. "
                f"Valid choices are: {', '.join(SUPPLY_TO_COLOR)}"
            )
            return

        self.set_state("WAITING_FOR_TARGET")
        self.get_logger().info(
            f"Accepted request '{self.requested_bin}'. "
            f"Expected color: {self.expected_color}"
        )

    def visible_callback(self, message: Bool):
        self.latest_visible = message.data

    def color_callback(self, message: String):
        self.latest_color = message.data

    def slot_callback(self, message: String):
        self.latest_slot = message.data

    def target_callback(self, target: PoseStamped):
        if self.state != "WAITING_FOR_TARGET":
            return

        if not self.latest_visible:
            return

        if self.latest_color != self.expected_color:
            self.get_logger().warn(
                f"Ignoring target with color '{self.latest_color}'. "
                f"Expected '{self.expected_color}'."
            )
            return

        if self.latest_slot == "none":
            self.get_logger().warn("Ignoring target with no valid shelf slot.")
            return

        self.set_state("TARGET_DETECTED")

        self.pick_target_pub.publish(target)
        self.pick_target_sent = True

        self.get_logger().info(
            f"Publishing pick target for '{self.requested_bin}' "
            f"at slot '{self.latest_slot}'."
        )

        self.set_state("WAITING_FOR_MANIPULATION")

    def manipulation_callback(self, message: String):
        manipulation_status = message.data

        if self.state != "WAITING_FOR_MANIPULATION":
            return

        if manipulation_status == "SUCCEEDED":
            self.set_state("COMPLETE")
            self.get_logger().info(
                f"Task complete. Retrieved '{self.requested_bin}' "
                f"from slot '{self.latest_slot}'."
            )

        elif manipulation_status == "FAILED":
            self.set_state("FAILED_MANIPULATION")
            self.get_logger().error(
                f"Manipulation failed for '{self.requested_bin}' "
                f"at slot '{self.latest_slot}'."
            )

    def set_state(self, new_state):
        if self.state == new_state:
            return

        self.state = new_state

        status_msg = String()
        status_msg.data = new_state
        self.task_status_pub.publish(status_msg)

        self.get_logger().info(f"Task state: {new_state}")


def main(args=None):
    rclpy.init(args=args)
    node = TaskManager()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

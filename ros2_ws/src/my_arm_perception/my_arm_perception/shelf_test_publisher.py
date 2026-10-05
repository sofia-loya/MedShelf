#!/usr/bin/env python3

import random
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from my_arm_perception.bin_config import SLOT_NAMES


BIN_COLORS_BGR = {
    "red": (0, 0, 255),
    "green": (0, 180, 0),
    "blue": (255, 0, 0),
    "yellow": (0, 255, 255),
    "orange": (0, 140, 255),
    "purple": (180, 0, 140),
    "pink": (180, 100, 255),
    "cyan": (255, 255, 0),
    "black": (20, 20, 20),
}


class ShelfTestPublisher(Node):
    def __init__(self):
        super().__init__("shelf_test_publisher")

        self.declare_parameter("seed", 0)
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("image_width", 640)
        self.declare_parameter("image_height", 480)
        self.declare_parameter("image_topic", "/medshelf/camera/image_raw")

        self.seed = int(self.get_parameter("seed").value)
        self.publish_rate_hz = float(
            self.get_parameter("publish_rate_hz").value
        )
        self.image_width = int(self.get_parameter("image_width").value)
        self.image_height = int(self.get_parameter("image_height").value)
        image_topic = self.get_parameter("image_topic").value

        self.bridge = CvBridge()
        self.image_pub = self.create_publisher(
            Image,
            image_topic,
            qos_profile_sensor_data,
        )

        self.slot_to_color = self.make_layout(self.seed)
        self.image = self.draw_shelf()

        period = 1.0 / self.publish_rate_hz
        self.timer = self.create_timer(period, self.publish_image)

        self.get_logger().info(
            f"Publishing synthetic shelf images on {image_topic}"
        )
        self.get_logger().info(
            f"Seed {self.seed} layout: {self.slot_to_color}"
        )

    def make_layout(self, seed):
        colors = list(BIN_COLORS_BGR.keys())
        random.Random(seed).shuffle(colors)

        return {
            slot_name: color_name
            for slot_name, color_name in zip(SLOT_NAMES, colors)
        }

    def draw_shelf(self):
        image = np.full(
            (self.image_height, self.image_width, 3),
            235,
            dtype=np.uint8,
        )

        shelf_margin_x = 50
        shelf_margin_y = 35
        shelf_width = self.image_width - 2 * shelf_margin_x
        shelf_height = self.image_height - 2 * shelf_margin_y

        cv2.rectangle(
            image,
            (shelf_margin_x, shelf_margin_y),
            (shelf_margin_x + shelf_width, shelf_margin_y + shelf_height),
            (120, 120, 120),
            thickness=-1,
        )

        cell_width = shelf_width // 3
        cell_height = shelf_height // 3

        for row in range(3):
            for column in range(3):
                slot_index = row * 3 + column
                slot_name = SLOT_NAMES[slot_index]
                color_name = self.slot_to_color[slot_name]

                x0 = shelf_margin_x + column * cell_width
                y0 = shelf_margin_y + row * cell_height
                x1 = x0 + cell_width
                y1 = y0 + cell_height

                cv2.rectangle(
                    image,
                    (x0 + 4, y0 + 4),
                    (x1 - 4, y1 - 4),
                    (245, 245, 245),
                    thickness=-1,
                )

                bin_margin_x = 24
                bin_margin_y = 20
                bin_x0 = x0 + bin_margin_x
                bin_y0 = y0 + bin_margin_y
                bin_x1 = x1 - bin_margin_x
                bin_y1 = y1 - bin_margin_y

                cv2.rectangle(
                    image,
                    (bin_x0, bin_y0),
                    (bin_x1, bin_y1),
                    BIN_COLORS_BGR[color_name],
                    thickness=-1,
                )

                cv2.rectangle(
                    image,
                    (bin_x0, bin_y0),
                    (bin_x1, bin_y1),
                    (40, 40, 40),
                    thickness=3,
                )

                handle_width = (bin_x1 - bin_x0) // 2
                handle_height = 18
                handle_x0 = (bin_x0 + bin_x1 - handle_width) // 2
                handle_y0 = bin_y1 - handle_height - 12

                cv2.rectangle(
                    image,
                    (handle_x0, handle_y0),
                    (handle_x0 + handle_width, handle_y0 + handle_height),
                    (55, 55, 55),
                    thickness=-1,
                )

                text_color = (255, 255, 255)
                if color_name in ("yellow", "cyan", "pink", "orange"):
                    text_color = (0, 0, 0)

                cv2.putText(
                    image,
                    color_name,
                    (bin_x0 + 8, bin_y0 + 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    text_color,
                    2,
                )

                cv2.putText(
                    image,
                    slot_name,
                    (x0 + 8, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.40,
                    (30, 30, 30),
                    1,
                )

        cv2.putText(
            image,
            f"MedShelf synthetic camera | seed={self.seed}",
            (15, 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (30, 30, 30),
            2,
        )

        return image

    def publish_image(self):
        image_msg = self.bridge.cv2_to_imgmsg(
            self.image,
            encoding="bgr8",
        )
        image_msg.header.stamp = self.get_clock().now().to_msg()
        image_msg.header.frame_id = "medshelf_camera_optical_frame"
        self.image_pub.publish(image_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ShelfTestPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from my_arm_perception.bin_config import HSV_RANGES, SLOT_NAMES, SUPPLY_TO_COLOR


class BinDetector(Node):
    def __init__(self):
        super().__init__("bin_detector")

        self.declare_parameter("requested_bin", "gauze")
        self.declare_parameter("image_topic", "/medshelf/camera/image_raw")
        self.declare_parameter("minimum_contour_area", 500.0)
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("required_stable_frames", 3)

        requested_bin = self.get_parameter("requested_bin").value
        if requested_bin not in SUPPLY_TO_COLOR:
            self.get_logger().warn(
                f"Unknown requested_bin '{requested_bin}'. "
                f"Valid choices are: {', '.join(SUPPLY_TO_COLOR)}"
            )

        image_topic = self.get_parameter("image_topic").value

        self.bridge = CvBridge()
        self.last_candidate = None
        self.stable_frame_count = 0
        self.confirmed_candidate = None

        self.annotated_pub = self.create_publisher(
            Image,
            "/medshelf/annotated_image",
            10,
        )
        self.slot_pub = self.create_publisher(
            String,
            "/medshelf/detected_slot",
            10,
        )
        self.color_pub = self.create_publisher(
            String,
            "/medshelf/detected_color",
            10,
        )
        self.visible_pub = self.create_publisher(
            Bool,
            "/medshelf/target_visible",
            10,
        )
        self.target_pub = self.create_publisher(
            PoseStamped,
            "/medshelf/detected_target",
            10,
        )

        self.image_sub = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            f"Bin detector started. Requested bin: {requested_bin}. "
            f"Required stable frames: "
            f"{self.get_parameter('required_stable_frames').value}. "
            f"Image topic: {image_topic}"
        )

    def image_callback(self, image_msg: Image):
        requested_bin = self.get_parameter("requested_bin").value

        if requested_bin not in SUPPLY_TO_COLOR:
            self.reset_detection()
            self.publish_not_visible(image_msg.header)
            return

        requested_color = SUPPLY_TO_COLOR[requested_bin]
        min_area = float(self.get_parameter("minimum_contour_area").value)

        try:
            image_bgr = self.bridge.imgmsg_to_cv2(
                image_msg,
                desired_encoding="bgr8",
            )
        except Exception as error:
            self.get_logger().error(f"Could not convert image: {error}")
            return

        annotated = image_bgr.copy()
        height, width = image_bgr.shape[:2]

        self.draw_grid(annotated)

        hsv_image = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
        mask = self.make_color_mask(hsv_image, requested_color)

        kernel = np.ones((5, 5), dtype=np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )

        valid_contours = [
            contour
            for contour in contours
            if cv2.contourArea(contour) >= min_area
        ]

        if not valid_contours:
            self.reset_detection()
            self.publish_not_visible(image_msg.header)

            cv2.putText(
                annotated,
                f"{requested_bin} not detected",
                (20, 35),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 0, 255),
                2,
            )

            self.publish_annotated(annotated, image_msg.header)
            return

        contour = max(valid_contours, key=cv2.contourArea)
        area = cv2.contourArea(contour)
        x, y, box_width, box_height = cv2.boundingRect(contour)

        moments = cv2.moments(contour)
        if moments["m00"] == 0:
            self.reset_detection()
            self.publish_not_visible(image_msg.header)
            self.publish_annotated(annotated, image_msg.header)
            return

        center_x = int(moments["m10"] / moments["m00"])
        center_y = int(moments["m01"] / moments["m00"])
        slot_name = self.pixel_to_slot(center_x, center_y, width, height)

        candidate = (requested_color, slot_name)
        self.update_stability(candidate)

        required_frames = int(
            self.get_parameter("required_stable_frames").value
        )
        is_confirmed = self.stable_frame_count >= required_frames

        cv2.rectangle(
            annotated,
            (x, y),
            (x + box_width, y + box_height),
            (0, 255, 0),
            2,
        )
        cv2.circle(annotated, (center_x, center_y), 6, (0, 255, 0), -1)

        status = "CONFIRMED" if is_confirmed else "CHECKING"
        status_color = (0, 255, 0) if is_confirmed else (0, 255, 255)

        cv2.putText(
            annotated,
            f"{requested_bin} | {requested_color} | {slot_name}",
            (20, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            status_color,
            2,
        )
        cv2.putText(
            annotated,
            f"{status} {self.stable_frame_count}/{required_frames} "
            f"| area={area:.0f}",
            (20, 65),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            status_color,
            2,
        )

        if is_confirmed:
            self.publish_detection(
                requested_color,
                slot_name,
                image_msg.header,
            )
        else:
            self.publish_not_visible(image_msg.header)

        self.publish_annotated(annotated, image_msg.header)

    def make_color_mask(self, hsv_image, color_name):
        mask = np.zeros(hsv_image.shape[:2], dtype=np.uint8)

        for lower, upper in HSV_RANGES[color_name]:
            lower_bound = np.array(lower, dtype=np.uint8)
            upper_bound = np.array(upper, dtype=np.uint8)
            mask |= cv2.inRange(hsv_image, lower_bound, upper_bound)

        return mask

    def pixel_to_slot(self, pixel_x, pixel_y, image_width, image_height):
        column = min(2, int(3 * pixel_x / image_width))
        row = min(2, int(3 * pixel_y / image_height))
        return SLOT_NAMES[row * 3 + column]

    def update_stability(self, candidate):
        if candidate == self.last_candidate:
            self.stable_frame_count += 1
        else:
            self.last_candidate = candidate
            self.stable_frame_count = 1
            self.confirmed_candidate = None

        required_frames = int(
            self.get_parameter("required_stable_frames").value
        )

        if self.stable_frame_count == required_frames:
            self.confirmed_candidate = candidate
            self.get_logger().info(
                f"Confirmed target: color={candidate[0]}, "
                f"slot={candidate[1]}"
            )

    def reset_detection(self):
        self.last_candidate = None
        self.stable_frame_count = 0
        self.confirmed_candidate = None

    def draw_grid(self, image):
        height, width = image.shape[:2]

        for column in (1, 2):
            x = int(column * width / 3)
            cv2.line(image, (x, 0), (x, height), (255, 255, 255), 2)

        for row in (1, 2):
            y = int(row * height / 3)
            cv2.line(image, (0, y), (width, y), (255, 255, 255), 2)

        for row in range(3):
            for column in range(3):
                slot_name = SLOT_NAMES[row * 3 + column]
                text_x = int((column + 0.03) * width / 3)
                text_y = int((row + 0.12) * height / 3)

                cv2.putText(
                    image,
                    slot_name,
                    (text_x, text_y),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.45,
                    (255, 255, 255),
                    1,
                )

    def publish_detection(self, color_name, slot_name, header):
        visible_msg = Bool()
        visible_msg.data = True
        self.visible_pub.publish(visible_msg)

        color_msg = String()
        color_msg.data = color_name
        self.color_pub.publish(color_msg)

        slot_msg = String()
        slot_msg.data = slot_name
        self.slot_pub.publish(slot_msg)

        target_msg = PoseStamped()
        target_msg.header.stamp = header.stamp
        target_msg.header.frame_id = self.get_parameter("world_frame").value
        target_msg.pose.orientation.w = 1.0
        self.target_pub.publish(target_msg)

    def publish_not_visible(self, header):
        visible_msg = Bool()
        visible_msg.data = False
        self.visible_pub.publish(visible_msg)

        slot_msg = String()
        slot_msg.data = "none"
        self.slot_pub.publish(slot_msg)

        color_msg = String()
        color_msg.data = "none"
        self.color_pub.publish(color_msg)

    def publish_annotated(self, image_bgr, header):
        output_msg = self.bridge.cv2_to_imgmsg(
            image_bgr,
            encoding="bgr8",
        )
        output_msg.header = header
        self.annotated_pub.publish(output_msg)


def main(args=None):
    rclpy.init(args=args)
    node = BinDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
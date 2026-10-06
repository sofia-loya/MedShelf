#!/usr/bin/env python3
"""Grab one frame from the shelf camera and report every tray-sized color blob.

Prints, for each blob: pixel centroid, bounding box, and median HSV (OpenCV scale),
plus how many pixels each bin_config.HSV_RANGES color matches inside that blob.
Saves the raw frame as camera_probe.png.

Run (Gazebo + camera bridge running):
  python3 camera_probe.py
"""
import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from my_arm_perception.bin_config import HSV_RANGES

TOPIC = "/medshelf/camera/image_raw"


class Probe(Node):
    def __init__(self):
        super().__init__("camera_probe")
        self.frame = None
        self.create_subscription(Image, TOPIC, self.cb, qos_profile_sensor_data)

    def cb(self, msg):
        if self.frame is None:
            self.frame = CvBridge().imgmsg_to_cv2(msg, desired_encoding="bgr8")


def color_mask(hsv, name):
    m = np.zeros(hsv.shape[:2], np.uint8)
    for lo, hi in HSV_RANGES[name]:
        m |= cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))
    return m


def main():
    rclpy.init()
    node = Probe()
    while rclpy.ok() and node.frame is None:
        rclpy.spin_once(node, timeout_sec=0.5)
    img = node.frame
    node.destroy_node()
    rclpy.shutdown()

    cv2.imwrite("camera_probe.png", img)
    h, w = img.shape[:2]
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # Tray candidates: clearly colored pixels, or very dark pixels (black tray).
    sat = (hsv[:, :, 1] > 70) & (hsv[:, :, 2] > 40)
    dark = hsv[:, :, 2] < 45
    cand = ((sat | dark) * 255).astype(np.uint8)
    cand = cv2.morphologyEx(cand, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))

    n, labels, stats, cents = cv2.connectedComponentsWithStats(cand)
    blobs = [i for i in range(1, n) if stats[i, cv2.CC_STAT_AREA] >= 300]
    blobs.sort(key=lambda i: (cents[i][1], cents[i][0]))

    print(f"Image {w}x{h}. Saved camera_probe.png. {len(blobs)} blobs:\n")
    print(f"{'cx':>4} {'cy':>4}  {'x0':>4} {'y0':>4} {'x1':>4} {'y1':>4}  {'area':>6}   "
          f"{'H':>3} {'S':>3} {'V':>3}   matches (pixels)")
    for i in blobs:
        x, y, bw, bh, area = stats[i]
        cx, cy = cents[i]
        region = labels == i
        H, S, V = (int(np.median(hsv[:, :, c][region])) for c in range(3))
        hits = {name: int(np.count_nonzero(color_mask(hsv, name)[region])) for name in HSV_RANGES}
        hits = {k: v for k, v in sorted(hits.items(), key=lambda kv: -kv[1]) if v > 0}
        print(f"{cx:4.0f} {cy:4.0f}  {x:4d} {y:4d} {x + bw:4d} {y + bh:4d}  {area:6d}   "
              f"{H:3d} {S:3d} {V:3d}   {hits}")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""One-shot stand-in for a camera: publish each image in a folder as a single
frame, wait for qr_match_node's verdict on that exact frame, print it, then
move to the next image. Exits once the folder is exhausted.

qr_match_node itself keeps running forever throughout, unmodified -- this
script only replaces the camera driver for testing without one.

Usage:
    ros2 run qr_match qr_match_test_feed /path/to/test_data
"""

from __future__ import annotations

import argparse
import glob
import os
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from sensor_msgs.msg import Image

from qr_match_interfaces.msg import QRResult


class TestImageFeed(Node):
    def __init__(self, image_dir, image_topic, result_topic, timeout, exclude):
        super().__init__("qr_match_test_feed")
        self.bridge = CvBridge()
        self.result = None
        self.timeout = timeout

        image_qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                               history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        result_qos = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
                                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                                history=QoSHistoryPolicy.KEEP_LAST, depth=1)

        self.pub = self.create_publisher(Image, image_topic, image_qos)
        self.create_subscription(QRResult, result_topic, self._on_result, result_qos)

        exts = ("*.png", "*.jpg", "*.jpeg")
        self.paths = sorted(
            p for ext in exts for p in glob.glob(os.path.join(image_dir, ext))
            if os.path.basename(p) not in exclude)

    def _on_result(self, msg: QRResult) -> None:
        self.result = msg

    def run(self) -> None:
        if not self.paths:
            print("No images found to test.")
            return
        for path in self.paths:
            frame = cv2.imread(path)
            if frame is None:
                print(f"{os.path.basename(path):30s} could not read image")
                continue

            self.result = None
            stamp = self.get_clock().now().to_msg()
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            msg.header.stamp = stamp

            # Publish on a loop (not just once) so a slow DDS discovery
            # handshake with qr_match_node's subscription can't drop it.
            deadline = time.monotonic() + self.timeout
            while time.monotonic() < deadline:
                self.pub.publish(msg)
                rclpy.spin_once(self, timeout_sec=0.2)
                if self.result is not None and self.result.header.stamp == stamp:
                    break

            if self.result is None or self.result.header.stamp != stamp:
                print(f"{os.path.basename(path):30s} NO RESPONSE "
                      f"(is qr_match_node running?)")
                continue

            r = self.result
            verdict = "MATCH" if r.match_found else "no match"
            print(f"{os.path.basename(path):30s} {verdict:9s} "
                  f"best={r.best_score:.3f} detections={r.num_detections} "
                  f"({r.processing_time_ms:.0f} ms)")


def main(args=None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("image_dir", help="Folder of test images to feed in.")
    parser.add_argument("--image-topic", default="/image_raw")
    parser.add_argument("--result-topic", default="/qr_match_node/result")
    parser.add_argument("--exclude", nargs="*", default=["reference_qr.png"],
                        help="Filenames in image_dir to skip (default: the reference).")
    parser.add_argument("--timeout", type=float, default=5.0,
                        help="Seconds to wait for a result per image.")
    parsed, ros_args = parser.parse_known_args(args)

    rclpy.init(args=ros_args)
    node = TestImageFeed(parsed.image_dir, parsed.image_topic, parsed.result_topic,
                         parsed.timeout, set(parsed.exclude))
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

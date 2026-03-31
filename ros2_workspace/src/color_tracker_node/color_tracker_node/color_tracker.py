#!/usr/bin/env python3
"""HSV-based red ball tracker with centroid + distance estimate."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32


@dataclass(frozen=True)
class TrackerConfig:
    image_topic: str
    min_radius_px: float
    expected_ball_diameter_m: float
    focal_length_px: float


class ColorTrackerNode(Node):
    """Track red ball and estimate relative distance."""

    def __init__(self) -> None:
        super().__init__("color_tracker_node")
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("min_radius_px", 10.0)
        self.declare_parameter("expected_ball_diameter_m", 0.12)
        self.declare_parameter("focal_length_px", 620.0)

        self._cfg = TrackerConfig(
            image_topic=str(self.get_parameter("image_topic").value),
            min_radius_px=float(self.get_parameter("min_radius_px").value),
            expected_ball_diameter_m=float(self.get_parameter("expected_ball_diameter_m").value),
            focal_length_px=float(self.get_parameter("focal_length_px").value),
        )

        self._bridge = CvBridge()
        self._centroid_pub = self.create_publisher(PointStamped, "/perception/ball_centroid", 10)
        self._conf_pub = self.create_publisher(Float32, "/perception/tracking_confidence", 10)
        self._debug_pub = self.create_publisher(Image, "/perception/debug/ball_image", 10)
        self.create_subscription(Image, self._cfg.image_topic, self._on_image, 10)

    def _on_image(self, msg: Image) -> None:
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            lower_red_1 = np.array([0, 120, 70])
            upper_red_1 = np.array([10, 255, 255])
            lower_red_2 = np.array([170, 120, 70])
            upper_red_2 = np.array([180, 255, 255])

            mask1 = cv2.inRange(hsv, lower_red_1, upper_red_1)
            mask2 = cv2.inRange(hsv, lower_red_2, upper_red_2)
            mask = cv2.bitwise_or(mask1, mask2)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            confidence_msg = Float32()
            confidence_msg.data = 0.0

            if contours:
                contour = max(contours, key=cv2.contourArea)
                (x, y), radius = cv2.minEnclosingCircle(contour)
                if radius >= self._cfg.min_radius_px:
                    confidence = min(1.0, float(radius / 80.0))
                    confidence_msg.data = confidence

                    ball_diameter_px = max(radius * 2.0, 1.0)
                    distance_m = (self._cfg.expected_ball_diameter_m * self._cfg.focal_length_px) / ball_diameter_px

                    point = PointStamped()
                    point.header.stamp = self.get_clock().now().to_msg()
                    point.header.frame_id = "camera_link"
                    point.point.x = float(x)
                    point.point.y = float(y)
                    point.point.z = float(distance_m)
                    self._centroid_pub.publish(point)

                    cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 0), 2)
                    cv2.putText(
                        frame,
                        f"dist={distance_m:.2f}m conf={confidence:.2f}",
                        (int(x) - 80, int(y) - 20),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0, 255, 0),
                        2,
                    )

            self._conf_pub.publish(confidence_msg)
            self._debug_pub.publish(self._bridge.cv2_to_imgmsg(frame, encoding="bgr8"))
        except Exception as exc:
            self.get_logger().error(f"Color tracker callback failed: {exc}")


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = ColorTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

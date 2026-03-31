#!/usr/bin/env python3
"""Phase 2 wheel odometry + TF publisher from motor feedback."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

import rclpy
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

from forest_rover_msgs.msg import MotorFeedback


@dataclass(frozen=True)
class OdomConfig:
    wheel_radius_m: float
    wheel_base_m: float
    encoder_cpr: int


class OdometryPublisherNode(Node):
    """Convert wheel feedback into odometry and odom->base_link transform."""

    def __init__(self) -> None:
        super().__init__("odometry_publisher")

        self.declare_parameter("wheel_radius_m", 0.05)
        self.declare_parameter("wheel_base_m", 0.26)
        self.declare_parameter("encoder_cpr", 464)

        self._cfg = OdomConfig(
            wheel_radius_m=float(self.get_parameter("wheel_radius_m").value),
            wheel_base_m=float(self.get_parameter("wheel_base_m").value),
            encoder_cpr=int(self.get_parameter("encoder_cpr").value),
        )

        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._last_left = 0
        self._last_right = 0
        self._has_prev_ticks = False
        self._last_stamp = self.get_clock().now()

        self._odom_pub = self.create_publisher(Odometry, "/odometry/raw", 50)
        self._tf_pub = TransformBroadcaster(self)
        self.create_subscription(MotorFeedback, "/raw_motor_feedback", self._on_feedback, 50)

        self.get_logger().info("odometry_publisher active")

    def _on_feedback(self, msg: MotorFeedback) -> None:
        stamp = self.get_clock().now()
        dt = (stamp - self._last_stamp).nanoseconds / 1e9
        self._last_stamp = stamp
        if dt <= 0.0:
            return

        if not self._has_prev_ticks:
            self._last_left = msg.left_encoder_ticks
            self._last_right = msg.right_encoder_ticks
            self._has_prev_ticks = True
            return

        left_delta = msg.left_encoder_ticks - self._last_left
        right_delta = msg.right_encoder_ticks - self._last_right
        self._last_left = msg.left_encoder_ticks
        self._last_right = msg.right_encoder_ticks

        left_dist = self._ticks_to_distance(left_delta)
        right_dist = self._ticks_to_distance(right_delta)
        dist = (left_dist + right_dist) / 2.0
        yaw_delta = (right_dist - left_dist) / max(self._cfg.wheel_base_m, 0.01)

        self._yaw += yaw_delta
        self._x += dist * math.cos(self._yaw)
        self._y += dist * math.sin(self._yaw)

        linear_vel = dist / dt
        angular_vel = yaw_delta / dt

        odom = Odometry()
        odom.header.stamp = stamp.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        odom.pose.pose.orientation = self._yaw_to_quaternion(self._yaw)
        odom.twist.twist.linear.x = linear_vel
        odom.twist.twist.angular.z = angular_vel
        odom.pose.covariance[0] = 0.03
        odom.pose.covariance[7] = 0.03
        odom.pose.covariance[35] = 0.08
        self._odom_pub.publish(odom)

        transform = TransformStamped()
        transform.header.stamp = odom.header.stamp
        transform.header.frame_id = "odom"
        transform.child_frame_id = "base_link"
        transform.transform.translation.x = self._x
        transform.transform.translation.y = self._y
        transform.transform.rotation = odom.pose.pose.orientation
        self._tf_pub.sendTransform(transform)

    def _ticks_to_distance(self, ticks: int) -> float:
        revolutions = ticks / max(float(self._cfg.encoder_cpr), 1.0)
        return revolutions * 2.0 * math.pi * self._cfg.wheel_radius_m

    @staticmethod
    def _yaw_to_quaternion(yaw: float) -> Quaternion:
        q = Quaternion()
        q.w = math.cos(yaw / 2.0)
        q.z = math.sin(yaw / 2.0)
        return q


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = OdometryPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

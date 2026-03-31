#!/usr/bin/env python3
"""Apply acceleration limits from planner cmd_vel to motor bridge cmd_vel."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


@dataclass(frozen=True)
class SmootherConfig:
    max_linear_accel: float
    max_angular_accel: float
    output_rate_hz: float


class CmdVelSmootherNode(Node):
    """Generate smooth velocity commands to avoid jerky motor response."""

    def __init__(self) -> None:
        super().__init__("cmd_vel_smoother")

        self.declare_parameter("max_linear_accel", 0.4)
        self.declare_parameter("max_angular_accel", 0.8)
        self.declare_parameter("output_rate_hz", 30.0)

        self._cfg = SmootherConfig(
            max_linear_accel=float(self.get_parameter("max_linear_accel").value),
            max_angular_accel=float(self.get_parameter("max_angular_accel").value),
            output_rate_hz=float(self.get_parameter("output_rate_hz").value),
        )

        self._target = Twist()
        self._current = Twist()
        self._last_time = self.get_clock().now()

        self._pub = self.create_publisher(Twist, "/cmd_vel", 20)
        self.create_subscription(Twist, "/cmd_vel_raw", self._on_target, 20)

        period = 1.0 / max(self._cfg.output_rate_hz, 1.0)
        self.create_timer(period, self._tick)
        self.get_logger().info("cmd_vel_smoother active")

    def _on_target(self, msg: Twist) -> None:
        self._target = msg

    def _tick(self) -> None:
        now = self.get_clock().now()
        dt = (now - self._last_time).nanoseconds / 1e9
        self._last_time = now
        if dt <= 0.0:
            return

        self._current.linear.x = self._ramp(
            self._current.linear.x,
            self._target.linear.x,
            self._cfg.max_linear_accel * dt,
        )
        self._current.angular.z = self._ramp(
            self._current.angular.z,
            self._target.angular.z,
            self._cfg.max_angular_accel * dt,
        )

        self._pub.publish(self._current)

    @staticmethod
    def _ramp(current: float, target: float, max_delta: float) -> float:
        if target > current + max_delta:
            return current + max_delta
        if target < current - max_delta:
            return current - max_delta
        return target


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = CmdVelSmootherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

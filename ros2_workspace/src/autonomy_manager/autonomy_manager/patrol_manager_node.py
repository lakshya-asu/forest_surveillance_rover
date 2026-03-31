#!/usr/bin/env python3
"""Simple waypoint patrol publisher for Phase 2 integration tests."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node


@dataclass(frozen=True)
class PatrolConfig:
    publish_rate_hz: float


class PatrolManagerNode(Node):
    """Publish cyclic patrol waypoints as a lightweight autonomy placeholder."""

    def __init__(self) -> None:
        super().__init__("patrol_manager")
        self.declare_parameter("publish_rate_hz", 1.0)

        self._cfg = PatrolConfig(
            publish_rate_hz=float(self.get_parameter("publish_rate_hz").value),
        )
        self._waypoints = [(0.0, 0.0), (2.5, 0.0), (2.5, 2.0), (0.0, 2.0)]
        self._index = 0

        self._pub = self.create_publisher(PointStamped, "/autonomy/target_waypoint", 10)
        self.create_timer(1.0 / max(self._cfg.publish_rate_hz, 1.0), self._tick)
        self.get_logger().info("patrol_manager active")

    def _tick(self) -> None:
        target = self._waypoints[self._index]
        self._index = (self._index + 1) % len(self._waypoints)

        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.point.x = target[0]
        msg.point.y = target[1]
        self._pub.publish(msg)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = PatrolManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

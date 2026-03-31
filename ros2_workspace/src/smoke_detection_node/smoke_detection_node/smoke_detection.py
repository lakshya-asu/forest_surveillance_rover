#!/usr/bin/env python3
"""Smoke threshold evaluator and alert publisher."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32

from forest_rover_msgs.msg import EnvironmentalData, RoverEvent


@dataclass(frozen=True)
class SmokeConfig:
    warning_ppm: float
    critical_ppm: float


class SmokeDetectionNode(Node):
    """Evaluate smoke ppm and publish warning/critical alerts."""

    def __init__(self) -> None:
        super().__init__("smoke_detection_node")
        self.declare_parameter("warning_ppm", 300.0)
        self.declare_parameter("critical_ppm", 500.0)

        self._cfg = SmokeConfig(
            warning_ppm=float(self.get_parameter("warning_ppm").value),
            critical_ppm=float(self.get_parameter("critical_ppm").value),
        )

        self._reading_pub = self.create_publisher(Float32, "/gas_sensor/reading", 10)
        self._alert_pub = self.create_publisher(Bool, "/alerts/smoke_detected", 10)
        self._event_pub = self.create_publisher(RoverEvent, "/events/rover", 20)
        self.create_subscription(EnvironmentalData, "/environmental/data", self._on_env, 20)
        self._last_alert_state = False

    def _on_env(self, msg: EnvironmentalData) -> None:
        reading = Float32()
        reading.data = msg.smoke_ppm
        self._reading_pub.publish(reading)

        alert = Bool()
        alert.data = msg.smoke_ppm >= self._cfg.warning_ppm
        self._alert_pub.publish(alert)

        if alert.data and not self._last_alert_state:
            event = RoverEvent()
            event.stamp = self.get_clock().now().to_msg()
            event.event_type = "smoke_detected"
            event.severity = "critical" if msg.smoke_ppm >= self._cfg.critical_ppm else "warning"
            event.message = f"Smoke level {msg.smoke_ppm:.1f}ppm"
            event.value = msg.smoke_ppm
            self._event_pub.publish(event)

        self._last_alert_state = alert.data


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = SmokeDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

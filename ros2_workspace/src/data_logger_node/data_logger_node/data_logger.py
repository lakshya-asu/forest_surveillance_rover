#!/usr/bin/env python3
"""Phase 4 structured event and telemetry logger to SQLite."""

from __future__ import annotations

import os
import sqlite3
from dataclasses import dataclass
from typing import Optional

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node

from forest_rover_msgs.msg import DetectionArray, EnvironmentalData, RoverEvent


@dataclass(frozen=True)
class LoggerConfig:
    db_path: str


class DataLoggerNode(Node):
    """Persist rover telemetry and events for post-mission analysis."""

    def __init__(self) -> None:
        super().__init__("data_logger_node")
        self.declare_parameter("db_path", "mission_logs/mission_events.db")

        cfg = LoggerConfig(db_path=str(self.get_parameter("db_path").value))
        self._cfg = cfg
        self._conn = self._open_db(cfg.db_path)

        self.create_subscription(EnvironmentalData, "/environmental/data", self._on_env, 20)
        self.create_subscription(DetectionArray, "/perception/detections", self._on_detection, 20)
        self.create_subscription(Odometry, "/odometry/raw", self._on_odometry, 20)
        self.create_subscription(RoverEvent, "/events/rover", self._on_event, 20)

    def _open_db(self, db_path: str) -> sqlite3.Connection:
        os.makedirs(os.path.dirname(db_path) or ".", exist_ok=True)
        conn = sqlite3.connect(db_path)
        try:
            cur = conn.cursor()
            cur.execute(
                """
                CREATE TABLE IF NOT EXISTS telemetry (
                    ts TEXT,
                    temperature_c REAL,
                    humidity_percent REAL,
                    pressure_hpa REAL,
                    smoke_ppm REAL,
                    motion_detected INTEGER
                )
                """
            )
            cur.execute(
                """
                CREATE TABLE IF NOT EXISTS detections (
                    ts TEXT,
                    source TEXT,
                    label TEXT,
                    confidence REAL,
                    distance_m REAL
                )
                """
            )
            cur.execute(
                """
                CREATE TABLE IF NOT EXISTS odometry (
                    ts TEXT,
                    x REAL,
                    y REAL,
                    yaw_rate REAL,
                    linear_vel REAL
                )
                """
            )
            cur.execute(
                """
                CREATE TABLE IF NOT EXISTS events (
                    ts TEXT,
                    event_type TEXT,
                    severity TEXT,
                    message TEXT,
                    value REAL
                )
                """
            )
            conn.commit()
        except sqlite3.Error as exc:
            self.get_logger().error(f"DB init failed: {exc}")
        return conn

    def _on_env(self, msg: EnvironmentalData) -> None:
        try:
            self._conn.execute(
                "INSERT INTO telemetry VALUES (?, ?, ?, ?, ?, ?)",
                (
                    str(self.get_clock().now().nanoseconds),
                    msg.temperature_c,
                    msg.humidity_percent,
                    msg.pressure_hpa,
                    msg.smoke_ppm,
                    int(msg.motion_detected),
                ),
            )
            self._conn.commit()
        except sqlite3.Error as exc:
            self.get_logger().error(f"Telemetry write failed: {exc}")

    def _on_detection(self, msg: DetectionArray) -> None:
        ts = str(self.get_clock().now().nanoseconds)
        try:
            for det in msg.detections:
                self._conn.execute(
                    "INSERT INTO detections VALUES (?, ?, ?, ?, ?)",
                    (ts, msg.source, det.label, det.confidence, det.distance_m),
                )
            self._conn.commit()
        except sqlite3.Error as exc:
            self.get_logger().error(f"Detection write failed: {exc}")

    def _on_odometry(self, msg: Odometry) -> None:
        try:
            self._conn.execute(
                "INSERT INTO odometry VALUES (?, ?, ?, ?, ?)",
                (
                    str(self.get_clock().now().nanoseconds),
                    msg.pose.pose.position.x,
                    msg.pose.pose.position.y,
                    msg.twist.twist.angular.z,
                    msg.twist.twist.linear.x,
                ),
            )
            self._conn.commit()
        except sqlite3.Error as exc:
            self.get_logger().error(f"Odometry write failed: {exc}")

    def _on_event(self, msg: RoverEvent) -> None:
        try:
            self._conn.execute(
                "INSERT INTO events VALUES (?, ?, ?, ?, ?)",
                (
                    str(self.get_clock().now().nanoseconds),
                    msg.event_type,
                    msg.severity,
                    msg.message,
                    msg.value,
                ),
            )
            self._conn.commit()
        except sqlite3.Error as exc:
            self.get_logger().error(f"Event write failed: {exc}")

    def destroy_node(self) -> bool:
        self._conn.close()
        return super().destroy_node()


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = DataLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

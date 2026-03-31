"""
Base Station Receiver Node - receives and logs LoRa telemetry from rover.

This node runs on a ground station computer to:
1. Receive LoRa heartbeat messages (simulated via ROS topics for now)
2. Log telemetry to SQLite
3. Publish visualization data
4. Generate live maps/dashboards

For field deployment, this would interface with a LoRa gateway (e.g., RFM95W USB adapter).
"""

import rclpy
from rclpy.node import Node

import sqlite3
import json
from datetime import datetime
from pathlib import Path
from typing import Optional

from std_msgs.msg import String
from forest_rover_msgs.msg import TelemetryHeartbeat, LoRaStatus


class BaseStationReceiverNode(Node):
    """Ground station for receiving and logging rover telemetry."""

    def __init__(self):
        super().__init__("base_station_receiver_node")

        # Database file
        self._db_path = Path.home() / "rover_telemetry_base_station.db"
        self._init_database()

        # Subscriptions for LoRa telemetry (will come from ROS network or LoRa gateway)
        self._sub_heartbeat = self.create_subscription(
            TelemetryHeartbeat, "/telemetry/heartbeat", self._on_heartbeat, qos_profile=rclpy.qos.QoSProfile(depth=10)
        )
        self._sub_lora_status = self.create_subscription(
            LoRaStatus, "/telemetry/lora_status", self._on_lora_status, qos_profile=rclpy.qos.QoSProfile(depth=10)
        )

        # Publisher for visualization/mapping
        self._pub_rover_location = self.create_publisher(String, "/basestation/rover_location", qos_profile=10)

        self.get_logger().info(f"Base Station Receiver initialized. Telemetry DB: {self._db_path}")

    def _init_database(self) -> None:
        """Initialize SQLite database for telemetry logging."""
        try:
            with sqlite3.connect(str(self._db_path)) as conn:
                cursor = conn.cursor()

                # Create telemetry log table
                cursor.execute(
                    """
                    CREATE TABLE IF NOT EXISTS heartbeat_log (
                        id INTEGER PRIMARY KEY AUTOINCREMENT,
                        timestamp DATETIME DEFAULT CURRENT_TIMESTAMP,
                        sequence_number INTEGER,
                        position_x REAL,
                        position_y REAL,
                        yaw REAL,
                        velocity_cmd REAL,
                        detection_count INTEGER,
                        battery_voltage REAL,
                        battery_percentage INTEGER,
                        smoke_alert BOOLEAN,
                        gas_sensor_ppm REAL,
                        autonomy_state TEXT,
                        waypoint_index INTEGER,
                        rssi_estimate INTEGER
                    )
                """
                )

                # Create LoRa status table
                cursor.execute(
                    """
                    CREATE TABLE IF NOT EXISTS lora_status_log (
                        id INTEGER PRIMARY KEY AUTOINCREMENT,
                        timestamp DATETIME DEFAULT CURRENT_TIMESTAMP,
                        rssi INTEGER,
                        snr REAL,
                        packets_transmitted INTEGER,
                        packets_ack_received INTEGER,
                        packets_lost INTEGER,
                        lora_mode TEXT,
                        frequency_mhz REAL,
                        spreading_factor INTEGER,
                        bandwidth_khz INTEGER,
                        estimated_distance_m REAL
                    )
                """
                )

                # Create alerts table
                cursor.execute(
                    """
                    CREATE TABLE IF NOT EXISTS alerts (
                        id INTEGER PRIMARY KEY AUTOINCREMENT,
                        timestamp DATETIME DEFAULT CURRENT_TIMESTAMP,
                        alert_type TEXT,
                        severity TEXT,
                        description TEXT
                    )
                """
                )

                conn.commit()
                self.get_logger().info(f"Database initialized: {self._db_path}")

        except sqlite3.Error as e:
            self.get_logger().error(f"Database initialization error: {e}")

    def _on_heartbeat(self, msg: TelemetryHeartbeat) -> None:
        """Process and log heartbeat message."""
        try:
            # Validate message fields
            if not (0 <= msg.battery_percentage <= 100):
                self.get_logger().warn(f"Invalid battery percentage: {msg.battery_percentage}")
                return
            if msg.detection_count < 0 or msg.detection_count > 255:
                self.get_logger().warn(f"Invalid detection count: {msg.detection_count}")
                return
            if msg.waypoint_index < 0 or msg.waypoint_index > 255:
                self.get_logger().warn(f"Invalid waypoint index: {msg.waypoint_index}")
                return

            # Log to database with connection context manager
            with sqlite3.connect(str(self._db_path)) as conn:
                cursor = conn.cursor()

                cursor.execute(
                    """
                    INSERT INTO heartbeat_log (
                        sequence_number, position_x, position_y, yaw, velocity_cmd,
                        detection_count, battery_voltage, battery_percentage,
                        smoke_alert, gas_sensor_ppm, autonomy_state, waypoint_index, rssi_estimate
                    ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                """,
                    (
                        msg.sequence_number,
                        msg.position_x,
                        msg.position_y,
                        msg.yaw,
                        msg.velocity_cmd,
                        msg.detection_count,
                        msg.battery_voltage,
                        msg.battery_percentage,
                        msg.smoke_alert,
                        msg.gas_sensor_ppm,
                        msg.autonomy_state,
                        msg.waypoint_index,
                        msg.rssi_estimate,
                    ),
                )

                conn.commit()

            # Publish location for visualization
            location_json = json.dumps(
                {
                    "x": msg.position_x,
                    "y": msg.position_y,
                    "yaw": msg.yaw,
                    "battery": msg.battery_percentage,
                    "state": msg.autonomy_state,
                    "smoke": msg.smoke_alert,
                    "detections": msg.detection_count,
                }
            )
            self._pub_rover_location.publish(String(data=location_json))

            # Alert on smoke
            if msg.smoke_alert:
                self._log_alert("smoke_detected", "CRITICAL", f"Smoke detected at ({msg.position_x:.2f}, {msg.position_y:.2f})")
                self.get_logger().warn(
                    f"SMOKE ALERT! Position: ({msg.position_x:.2f}, {msg.position_y:.2f}), "
                    f"Gas: {msg.gas_sensor_ppm:.1f} ppm"
                )

            # Log low battery
            if msg.battery_percentage < 20:
                self._log_alert("low_battery", "WARNING", f"Battery low: {msg.battery_percentage}%")
                self.get_logger().warn(f"LOW BATTERY! {msg.battery_percentage}%")

            self.get_logger().debug(
                f"Heartbeat #{msg.sequence_number}: "
                f"({msg.position_x:.2f}, {msg.position_y:.2f}), "
                f"detections={msg.detection_count}, battery={msg.battery_percentage}%"
            )

        except sqlite3.Error as e:
            self.get_logger().error(f"Heartbeat logging error: {e}")
        except Exception as e:
            self.get_logger().error(f"Heartbeat callback error: {e}")

    def _on_lora_status(self, msg: LoRaStatus) -> None:
        """Process and log LoRa status."""
        try:
            with sqlite3.connect(str(self._db_path)) as conn:
                cursor = conn.cursor()

                cursor.execute(
                    """
                    INSERT INTO lora_status_log (
                        rssi, snr, packets_transmitted, packets_ack_received, packets_lost,
                        lora_mode, frequency_mhz, spreading_factor, bandwidth_khz, estimated_distance_m
                    ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                """,
                    (
                        msg.rssi,
                        msg.snr,
                        msg.packets_transmitted,
                        msg.packets_ack_received,
                        msg.packets_lost,
                        msg.lora_mode,
                        msg.frequency_mhz,
                        msg.spreading_factor,
                        msg.bandwidth_khz,
                        msg.estimated_distance_m,
                    ),
                )

                conn.commit()

            self.get_logger().debug(
                f"LoRa status: RSSI={msg.rssi}, SNR={msg.snr:.1f}dB, "
                f"distance≈{msg.estimated_distance_m:.0f}m, "
                f"packets: tx={msg.packets_transmitted} ack={msg.packets_ack_received}"
            )

        except sqlite3.Error as e:
            self.get_logger().error(f"LoRa status logging error: {e}")
        except Exception as e:
            self.get_logger().error(f"LoRa status callback error: {e}")

    def _log_alert(self, alert_type: str, severity: str, description: str) -> None:
        """Log an alert to the database."""
        try:
            with sqlite3.connect(str(self._db_path)) as conn:
                cursor = conn.cursor()

                cursor.execute(
                    """
                    INSERT INTO alerts (alert_type, severity, description)
                    VALUES (?, ?, ?)
                """,
                    (alert_type, severity, description),
                )

                conn.commit()

        except sqlite3.Error as e:
            self.get_logger().error(f"Alert logging error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = BaseStationReceiverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

"""
Telemetry Gateway Node - manages LoRa heartbeat transmission and alert prioritization.

Subscribes to:
  - /odometry/filtered (nav_msgs/Odometry)
  - /perception/detections (forest_rover_msgs/DetectionArray)
  - /gas_sensor/reading (forest_rover_msgs/EnvironmentalData)
  - /alerts/smoke_detected (forest_rover_msgs/RoverEvent)
  - /rover/autonomy_state (std_msgs/String)
  - /rover/battery (sensor_msgs/BatteryState, if available)

Publishes:
  - /telemetry/heartbeat (forest_rover_msgs/TelemetryHeartbeat) every 10s
  - /telemetry/lora_status (forest_rover_msgs/LoRaStatus)
"""

import rclpy
from rclpy.node import Node
from rclpy.timer import Timer

import math
import threading
from datetime import datetime
from typing import Optional

from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String
from forest_rover_msgs.msg import (
    DetectionArray,
    EnvironmentalData,
    RoverEvent,
    TelemetryHeartbeat,
    LoRaStatus,
)


class TelemetryGatewayNode(Node):
    """Manages LoRa heartbeat/telemetry transmission."""

    def __init__(self):
        super().__init__("telemetry_gateway_node")

        # State tracking with thread safety
        self._state_lock = threading.Lock()
        self._last_odometry: Optional[Odometry] = None
        self._total_detections = 0
        self._smoke_alert = False
        self._latest_gas_ppm = 0.0
        self._autonomy_state = "idle"
        self._waypoint_index = 0
        self._battery_voltage = 0.0
        self._battery_percentage = 100

        # Telemetry sequencing
        self._heartbeat_sequence = 0

        # Subscriptions
        self._sub_odometry = self.create_subscription(
            Odometry, "/odometry/filtered", self._on_odometry, qos_profile=rclpy.qos.QoSProfile(depth=1)
        )
        self._sub_detections = self.create_subscription(
            DetectionArray, "/perception/detections", self._on_detections, qos_profile=rclpy.qos.QoSProfile(depth=5)
        )
        self._sub_gas = self.create_subscription(
            EnvironmentalData, "/gas_sensor/reading", self._on_gas, qos_profile=rclpy.qos.QoSProfile(depth=1)
        )
        self._sub_smoke_alert = self.create_subscription(
            RoverEvent, "/alerts/smoke_detected", self._on_smoke_alert, qos_profile=rclpy.qos.QoSProfile(depth=5)
        )
        self._sub_autonomy = self.create_subscription(
            String, "/rover/autonomy_state", self._on_autonomy_state, qos_profile=rclpy.qos.QoSProfile(depth=1)
        )
        self._sub_battery = self.create_subscription(
            BatteryState, "/rover/battery", self._on_battery, qos_profile=rclpy.qos.QoSProfile(depth=1)
        )

        # Publishers
        self._pub_heartbeat = self.create_publisher(TelemetryHeartbeat, "/telemetry/heartbeat", qos_profile=10)
        self._pub_lora_status = self.create_publisher(LoRaStatus, "/telemetry/lora_status", qos_profile=10)

        # Heartbeat timer (10 second interval)
        self._heartbeat_timer: Optional[Timer] = None
        self._heartbeat_interval_sec = 10.0

        self._heartbeat_timer = self.create_timer(self._heartbeat_interval_sec, self._send_heartbeat)

        self.get_logger().info("TelemetryGatewayNode initialized")

    def _on_odometry(self, msg: Odometry) -> None:
        """Store latest odometry for heartbeat."""
        try:
            with self._state_lock:
                self._last_odometry = msg
        except Exception as e:
            self.get_logger().error(f"Odometry callback error: {e}")

    def _on_detections(self, msg: DetectionArray) -> None:
        """Count detections and track consecutive animals seen."""
        try:
            with self._state_lock:
                self._total_detections += len(msg.detections)
        except Exception as e:
            self.get_logger().error(f"Detection callback error: {e}")

    def _on_gas(self, msg: EnvironmentalData) -> None:
        """Store latest gas sensor reading."""
        try:
            with self._state_lock:
                self._latest_gas_ppm = msg.gas_ppm
        except Exception as e:
            self.get_logger().error(f"Gas sensor callback error: {e}")

    def _on_smoke_alert(self, msg: RoverEvent) -> None:
        """Track smoke alert status."""
        try:
            with self._state_lock:
                if msg.event_type == "smoke_detected":
                    self._smoke_alert = True
                    self.get_logger().info("Smoke alert received - will include in next heartbeat")
                elif msg.event_type == "smoke_cleared":
                    self._smoke_alert = False
                    self.get_logger().info("Smoke alert cleared")
        except Exception as e:
            self.get_logger().error(f"Smoke alert callback error: {e}")

    def _on_autonomy_state(self, msg: String) -> None:
        """Track current autonomy state."""
        try:
            with self._state_lock:
                self._autonomy_state = msg.data
        except Exception as e:
            self.get_logger().error(f"Autonomy state callback error: {e}")

    def _on_battery(self, msg: BatteryState) -> None:
        """Store battery information."""
        try:
            with self._state_lock:
                self._battery_voltage = msg.voltage
                self._battery_percentage = int(msg.percentage * 100)
        except Exception as e:
            self.get_logger().error(f"Battery callback error: {e}")

    def _send_heartbeat(self) -> None:
        """Compose and publish heartbeat message every 10 seconds."""
        try:
            now = self.get_clock().now()

            # Compose heartbeat with thread-safe state access
            heartbeat = TelemetryHeartbeat()
            heartbeat.timestamp = now.to_msg()
            
            with self._state_lock:
                heartbeat.sequence_number = self._heartbeat_sequence
                self._heartbeat_sequence += 1

                # Position & orientation
                if self._last_odometry is not None:
                    heartbeat.position_x = self._last_odometry.pose.pose.position.x
                    heartbeat.position_y = self._last_odometry.pose.pose.position.y

                    # Extract yaw from quaternion
                    quat = self._last_odometry.pose.pose.orientation
                    yaw = self._quaternion_to_yaw(quat.x, quat.y, quat.z, quat.w)
                    heartbeat.yaw = yaw

                    # Velocity
                    heartbeat.velocity_cmd = self._last_odometry.twist.twist.linear.x
                else:
                    heartbeat.position_x = 0.0
                    heartbeat.position_y = 0.0
                    heartbeat.yaw = 0.0
                    heartbeat.velocity_cmd = 0.0

                # Detection & environmental
                heartbeat.detection_count = self._total_detections
                heartbeat.battery_voltage = self._battery_voltage
                heartbeat.battery_percentage = self._battery_percentage

                # Alerts
                heartbeat.smoke_alert = self._smoke_alert
                heartbeat.gas_sensor_ppm = self._latest_gas_ppm

                # Autonomy
                heartbeat.autonomy_state = self._autonomy_state
                heartbeat.waypoint_index = self._waypoint_index

            # RSSI placeholder (will be filled in by LoRa driver)
            heartbeat.rssi_estimate = 200  # Placeholder

            self._pub_heartbeat.publish(heartbeat)
            self.get_logger().debug(
                f"Heartbeat #{heartbeat.sequence_number}: "
                f"pos=({heartbeat.position_x:.2f}, {heartbeat.position_y:.2f}), "
                f"detections={heartbeat.detection_count}, "
                f"battery={heartbeat.battery_percentage}%, "
                f"smoke={heartbeat.smoke_alert}"
            )

        except Exception as e:
            self.get_logger().error(f"Heartbeat send error: {e}")

    @staticmethod
    def _quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
        """Convert quaternion to yaw angle (radians)."""
        sin_roll = 2.0 * (w * x + y * z)
        cos_roll = 1.0 - 2.0 * (x * x + y * y)
        _roll = math.atan2(sin_roll, cos_roll)

        sin_pitch = 2.0 * (w * y - z * x)
        sin_pitch = max(-1.0, min(1.0, sin_pitch))
        _pitch = math.asin(sin_pitch)

        sin_yaw = 2.0 * (w * z + x * y)
        cos_yaw = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(sin_yaw, cos_yaw)

        return yaw


def main(args=None):
    rclpy.init(args=args)
    node = TelemetryGatewayNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

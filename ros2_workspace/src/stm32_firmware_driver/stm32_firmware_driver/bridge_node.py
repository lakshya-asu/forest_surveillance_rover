#!/usr/bin/env python3
"""STM32 serial bridge for Phase 1 hardware integration.

This node supports both real serial mode and simulation mode.
"""

from __future__ import annotations

import random
import struct
import time
from dataclasses import dataclass
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool

from forest_rover_msgs.msg import EnvironmentalData, MotorCommand, MotorFeedback
from forest_rover_msgs.srv import EmergencyStop

try:
    import serial
except ImportError:  # pragma: no cover - optional runtime dependency
    serial = None


@dataclass(frozen=True)
class BridgeConfig:
    serial_port: str
    baud_rate: int
    frame_rate_hz: float
    simulate: bool
    wheel_base_m: float


def _crc16(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def _cobs_encode(raw: bytes) -> bytes:
    out = bytearray()
    code_idx = 0
    out.append(0)
    code = 1

    for byte in raw:
        if byte == 0:
            out[code_idx] = code
            code_idx = len(out)
            out.append(0)
            code = 1
        else:
            out.append(byte)
            code += 1
            if code == 0xFF:
                out[code_idx] = code
                code_idx = len(out)
                out.append(0)
                code = 1

    out[code_idx] = code
    return bytes(out)


def _cobs_decode(encoded: bytes) -> bytes:
    out = bytearray()
    i = 0
    while i < len(encoded):
        code = encoded[i]
        i += 1
        if code == 0:
            raise ValueError("Invalid COBS frame")

        for _ in range(1, code):
            if i >= len(encoded):
                raise ValueError("Truncated COBS frame")
            out.append(encoded[i])
            i += 1

        if code != 0xFF and i < len(encoded):
            out.append(0)
    return bytes(out)


class Stm32BridgeNode(Node):
    """Bridge ROS topics/services with STM32 firmware transport."""

    def __init__(self) -> None:
        super().__init__("stm32_firmware_bridge")

        self.declare_parameter("serial_port", "/dev/ttyUSB0")
        self.declare_parameter("baud_rate", 115200)
        self.declare_parameter("frame_rate_hz", 50.0)
        self.declare_parameter("simulate", True)
        self.declare_parameter("wheel_base_m", 0.26)

        cfg = BridgeConfig(
            serial_port=self.get_parameter("serial_port").value,
            baud_rate=int(self.get_parameter("baud_rate").value),
            frame_rate_hz=float(self.get_parameter("frame_rate_hz").value),
            simulate=bool(self.get_parameter("simulate").value),
            wheel_base_m=float(self.get_parameter("wheel_base_m").value),
        )
        self._config = cfg

        self._serial = self._open_serial(cfg)
        self._left_rpm = 0.0
        self._right_rpm = 0.0
        self._left_ticks = 0
        self._right_ticks = 0
        self._last_heartbeat = time.monotonic()
        self._watchdog_triggered = False
        self._estop_latched = False
        self._serial_rx_buffer = bytearray()

        self._env_pub = self.create_publisher(EnvironmentalData, "/environmental/data", 10)
        self._imu_pub = self.create_publisher(Imu, "/imu/data", 10)
        self._motor_pub = self.create_publisher(MotorFeedback, "/raw_motor_feedback", 20)
        self._odom_pub = self.create_publisher(Odometry, "/odometry/raw", 20)
        self._safety_pub = self.create_publisher(Bool, "/safety/watchdog", 10)

        self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, 20)
        self._estop_srv = self.create_service(EmergencyStop, "/emergency_stop", self._on_emergency_stop)

        period = 1.0 / max(cfg.frame_rate_hz, 1.0)
        self._timer = self.create_timer(period, self._tick)
        mode = "simulation" if cfg.simulate else "serial"
        self.get_logger().info(f"STM32 bridge started ({mode} mode)")

    def _open_serial(self, cfg: BridgeConfig):
        if cfg.simulate:
            return None
        if serial is None:
            self.get_logger().warning("pyserial unavailable, falling back to simulation mode")
            return None
        try:
            return serial.Serial(cfg.serial_port, cfg.baud_rate, timeout=0.01)
        except Exception as exc:  # pragma: no cover - runtime path
            self.get_logger().warning(f"Serial open failed: {exc}; switching to simulation")
            return None

    def _on_cmd_vel(self, msg: Twist) -> None:
        if self._estop_latched:
            return

        linear = msg.linear.x
        angular = msg.angular.z
        half_base = self._config.wheel_base_m / 2.0
        left_mps = linear - (angular * half_base)
        right_mps = linear + (angular * half_base)

        self._left_rpm = left_mps * 60.0 / (2.0 * 3.141592653589793 * 0.05)
        self._right_rpm = right_mps * 60.0 / (2.0 * 3.141592653589793 * 0.05)
        self._last_heartbeat = time.monotonic()
        self._watchdog_triggered = False

        cmd = MotorCommand()
        cmd.stamp = self.get_clock().now().to_msg()
        cmd.left_rpm = float(self._left_rpm)
        cmd.right_rpm = float(self._right_rpm)
        cmd.emergency_stop = False
        payload = struct.pack("<ff", cmd.left_rpm, cmd.right_rpm)
        self._send_frame(0x01, payload)

    def _on_emergency_stop(self, request: EmergencyStop.Request, response: EmergencyStop.Response):
        if request.stop:
            self._estop_latched = True
            self._left_rpm = 0.0
            self._right_rpm = 0.0
            self._watchdog_triggered = True
            self._send_frame(0x02, b"\x01")
            response.success = True
            response.message = "Emergency stop asserted"
        else:
            self._estop_latched = False
            self._watchdog_triggered = False
            self._send_frame(0x02, b"\x00")
            response.success = True
            response.message = "Emergency stop cleared"
        return response

    def _tick(self) -> None:
        now = time.monotonic()
        if now - self._last_heartbeat > 0.5:
            self._left_rpm = 0.0
            self._right_rpm = 0.0
            self._watchdog_triggered = True
        else:
            self._watchdog_triggered = False

        if self._estop_latched:
            self._left_rpm = 0.0
            self._right_rpm = 0.0

        if self._serial is not None:
            self._poll_serial()
        else:
            self._simulate_inputs()

        self._publish_feedback()

    def _poll_serial(self) -> None:
        read_count = self._serial.in_waiting if self._serial.in_waiting > 0 else 1
        incoming = self._serial.read(read_count)
        if not incoming:
            return

        self._serial_rx_buffer.extend(incoming)
        while True:
            try:
                sep_idx = self._serial_rx_buffer.index(0)
            except ValueError:
                break

            encoded = bytes(self._serial_rx_buffer[:sep_idx])
            del self._serial_rx_buffer[: sep_idx + 1]
            if not encoded:
                continue

            try:
                decoded = _cobs_decode(encoded)
            except ValueError:
                continue

            self._handle_frame(decoded)

    def _handle_frame(self, frame: bytes) -> None:
        if len(frame) < 5:
            return

        msg_id = frame[0]
        payload_len = frame[1] | (frame[2] << 8)
        if len(frame) != payload_len + 5:
            return

        payload = frame[3 : 3 + payload_len]
        rx_crc = frame[3 + payload_len] | (frame[4 + payload_len] << 8)
        calc_crc = _crc16(frame[: 3 + payload_len])
        if rx_crc != calc_crc:
            return

        if msg_id == 0x20 and payload_len >= 17:
            temperature, humidity, pressure, smoke = struct.unpack("<ffff", payload[:16])
            motion = bool(payload[16])

            env_msg = EnvironmentalData()
            env_msg.stamp = self.get_clock().now().to_msg()
            env_msg.temperature_c = float(temperature)
            env_msg.humidity_percent = float(humidity)
            env_msg.pressure_hpa = float(pressure)
            env_msg.smoke_ppm = float(smoke)
            env_msg.motion_detected = motion
            self._env_pub.publish(env_msg)

    def _simulate_inputs(self) -> None:
        self._left_ticks += int(self._left_rpm / 6.0)
        self._right_ticks += int(self._right_rpm / 6.0)

        env_msg = EnvironmentalData()
        env_msg.stamp = self.get_clock().now().to_msg()
        env_msg.temperature_c = 24.0 + random.uniform(-1.0, 1.0)
        env_msg.humidity_percent = 50.0 + random.uniform(-5.0, 5.0)
        env_msg.pressure_hpa = 1012.0 + random.uniform(-2.0, 2.0)
        env_msg.smoke_ppm = max(0.0, random.uniform(0.0, 25.0))
        env_msg.motion_detected = random.random() > 0.97
        self._env_pub.publish(env_msg)

        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "imu_link"
        imu_msg.orientation.w = 1.0
        imu_msg.angular_velocity.z = (self._right_rpm - self._left_rpm) * 0.001
        imu_msg.linear_acceleration.x = (self._left_rpm + self._right_rpm) * 0.0005
        self._imu_pub.publish(imu_msg)

    def _publish_feedback(self) -> None:
        stamp = self.get_clock().now().to_msg()

        motor_msg = MotorFeedback()
        motor_msg.stamp = stamp
        motor_msg.left_rpm = float(self._left_rpm)
        motor_msg.right_rpm = float(self._right_rpm)
        motor_msg.left_encoder_ticks = int(self._left_ticks)
        motor_msg.right_encoder_ticks = int(self._right_ticks)
        motor_msg.battery_voltage = 11.8
        motor_msg.watchdog_triggered = self._watchdog_triggered
        self._motor_pub.publish(motor_msg)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.twist.twist.linear.x = ((self._left_rpm + self._right_rpm) / 2.0) * (2.0 * 3.141592653589793 * 0.05 / 60.0)
        odom.twist.twist.angular.z = ((self._right_rpm - self._left_rpm) * 2.0 * 3.141592653589793 * 0.05 / 60.0) / max(
            self._config.wheel_base_m,
            0.01,
        )
        odom.twist.covariance[0] = 0.02
        odom.twist.covariance[35] = 0.05
        self._odom_pub.publish(odom)

        safety = Bool()
        safety.data = self._watchdog_triggered
        self._safety_pub.publish(safety)

    def _send_frame(self, msg_id: int, payload: bytes) -> None:
        if self._serial is None:
            return

        header = bytes([msg_id, len(payload) & 0xFF, (len(payload) >> 8) & 0xFF])
        crc = _crc16(header + payload)
        raw = header + payload + bytes([crc & 0xFF, (crc >> 8) & 0xFF])
        encoded = _cobs_encode(raw) + b"\x00"
        self._serial.write(encoded)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = Stm32BridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

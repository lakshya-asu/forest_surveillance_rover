#!/usr/bin/env python3
"""YOLOv8 detector node with simulation fallback.

Publishes DetectionArray on /perception/detections.
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Optional

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32

from forest_rover_msgs.msg import Detection, DetectionArray

try:
    from ultralytics import YOLO
except Exception:  # pragma: no cover
    YOLO = None


@dataclass(frozen=True)
class DetectorConfig:
    model_path: str
    confidence_threshold: float
    nms_iou_threshold: float
    image_topic: str
    simulate: bool


class YoloDetectorNode(Node):
    """YOLOv8 detector for forest animals."""

    def __init__(self) -> None:
        super().__init__("yolo_detector_node")
        self.declare_parameter("model_path", "yolov8n.pt")
        self.declare_parameter("confidence_threshold", 0.5)
        self.declare_parameter("nms_iou_threshold", 0.45)
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("simulate", False)

        self._cfg = DetectorConfig(
            model_path=str(self.get_parameter("model_path").value),
            confidence_threshold=float(self.get_parameter("confidence_threshold").value),
            nms_iou_threshold=float(self.get_parameter("nms_iou_threshold").value),
            image_topic=str(self.get_parameter("image_topic").value),
            simulate=bool(self.get_parameter("simulate").value),
        )

        self._bridge = CvBridge()
        self._detections_pub = self.create_publisher(DetectionArray, "/perception/detections", 10)
        self._fps_pub = self.create_publisher(Float32, "/perception/inference_fps", 10)
        self._debug_pub = self.create_publisher(Image, "/perception/debug/yolo_image", 10)
        self.create_subscription(Image, self._cfg.image_topic, self._on_image, 10)

        self._last_infer_ts = time.monotonic()
        self._model = None
        self._model_loaded = False
        self._load_model()

    def _load_model(self) -> None:
        if self._cfg.simulate:
            self.get_logger().info("YOLO detector in simulation mode")
            self._model_loaded = False
            return

        if YOLO is None:
            self.get_logger().warning("ultralytics not available; fallback to simulation")
            self._model_loaded = False
            return

        try:
            self._model = YOLO(self._cfg.model_path)
            self._model_loaded = True
            self.get_logger().info(f"Loaded YOLO model: {self._cfg.model_path}")
        except Exception as exc:
            self.get_logger().warning(f"Failed to load model ({exc}); fallback to simulation")
            self._model_loaded = False

    def _on_image(self, msg: Image) -> None:
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            detections_msg = DetectionArray()
            detections_msg.stamp = self.get_clock().now().to_msg()
            detections_msg.source = "yolo_detector"

            if self._model_loaded and self._model is not None:
                results = self._model.predict(
                    frame,
                    conf=self._cfg.confidence_threshold,
                    iou=self._cfg.nms_iou_threshold,
                    verbose=False,
                )
                for result in results:
                    names = result.names
                    for box in result.boxes:
                        conf = float(box.conf[0])
                        cls_idx = int(box.cls[0])
                        x1, y1, x2, y2 = box.xyxy[0].tolist()
                        det = Detection()
                        det.label = str(names.get(cls_idx, "unknown"))
                        det.confidence = conf
                        det.x_min = int(x1)
                        det.y_min = int(y1)
                        det.x_max = int(x2)
                        det.y_max = int(y2)
                        det.distance_m = -1.0
                        detections_msg.detections.append(det)
            else:
                h, w = frame.shape[:2]
                if int(time.time()) % 2 == 0:
                    det = Detection()
                    det.label = "deer"
                    det.confidence = 0.62
                    det.x_min = int(w * 0.35)
                    det.y_min = int(h * 0.3)
                    det.x_max = int(w * 0.65)
                    det.y_max = int(h * 0.85)
                    det.distance_m = 4.2
                    detections_msg.detections.append(det)

            self._detections_pub.publish(detections_msg)

            now = time.monotonic()
            dt = max(now - self._last_infer_ts, 1e-6)
            self._last_infer_ts = now
            fps = Float32()
            fps.data = float(1.0 / dt)
            self._fps_pub.publish(fps)

            self._debug_pub.publish(self._bridge.cv2_to_imgmsg(frame, encoding="bgr8"))
        except Exception as exc:
            self.get_logger().error(f"YOLO callback failed: {exc}")


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = YoloDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

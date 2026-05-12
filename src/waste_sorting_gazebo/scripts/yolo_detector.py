#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
YOLOv8 detector node for the waste sorting robot.

Inputs:
  - /top_camera/image_raw                sensor_msgs/Image

Outputs:
  - /waste/detections                    waste_sorting_gazebo/Detection2DArray
  - /waste/detection                     waste_sorting_gazebo/Detection2D
  - /waste/detection_image               sensor_msgs/Image

Purpose:
  This node converts Gazebo camera images into YOLO detections.

Important production behavior:
  - Publishes ALL valid detections on /waste/detections.
  - Publishes the best detection on /waste/detection for legacy compatibility.
  - Publishes annotated image for debugging.
  - Normalizes class names to: glass, metal, paper, plastic.
  - Does not crash silently.
"""

import os
import sys
import traceback
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Header

from waste_sorting_gazebo.msg import Detection2D, Detection2DArray


try:
    from ultralytics import YOLO
except Exception as exc:
    YOLO = None
    ULTRALYTICS_IMPORT_ERROR = exc
else:
    ULTRALYTICS_IMPORT_ERROR = None


VALID_CLASSES = {"glass", "metal", "paper", "plastic"}


def normalize_class_name(raw_name: str) -> Optional[str]:
    """
    Converts model class names into the exact 4 classes used by the project.

    Accepted final class names:
      - glass
      - metal
      - paper
      - plastic
    """

    if raw_name is None:
        return None

    name = str(raw_name).strip().lower()

    aliases: Dict[str, str] = {
        "glass": "glass",
        "bottle_glass": "glass",
        "glass_bottle": "glass",
        "green-glass": "glass",
        "brown-glass": "glass",
        "white-glass": "glass",

        "metal": "metal",
        "can": "metal",
        "metal_can": "metal",
        "aluminium": "metal",
        "aluminum": "metal",
        "tin": "metal",

        "paper": "paper",
        "cardboard": "paper",
        "paperboard": "paper",
        "carton": "paper",

        "plastic": "plastic",
        "plastic_bottle": "plastic",
        "bottle_plastic": "plastic",
        "pet": "plastic",
        "trash_plastic": "plastic",
    }

    if name in aliases:
        return aliases[name]

    # If model labels contain the expected word inside a longer label.
    for cls in VALID_CLASSES:
        if cls in name:
            return cls

    # Common fallback for models trained with recyclable material names.
    if "bottle" in name:
        return "plastic"
    if "can" in name:
        return "metal"
    if "cardboard" in name:
        return "paper"

    return None


class YoloDetectorNode:
    def __init__(self) -> None:
        rospy.init_node("yolo_detector", anonymous=False)

        self.image_topic: str = rospy.get_param("~image_topic", "/top_camera/image_raw")
        self.model_path: str = rospy.get_param(
            "~model_path",
            os.path.join(
                self._get_package_path_fallback(),
                "models_yolo",
                "best.pt",
            ),
        )
        self.conf_threshold: float = float(rospy.get_param("~conf_threshold", 0.25))
        self.iou_threshold: float = float(rospy.get_param("~iou_threshold", 0.45))
        self.publish_annotated: bool = bool(rospy.get_param("~publish_annotated", True))
        self.device: str = str(rospy.get_param("~device", "cpu"))

        self.detections_topic: str = rospy.get_param("~detections_topic", "/waste/detections")
        self.legacy_detection_topic: str = rospy.get_param("~legacy_detection_topic", "/waste/detection")
        self.detection_image_topic: str = rospy.get_param("~detection_image_topic", "/waste/detection_image")

        self.bridge = CvBridge()
        self.model = None
        self.class_names: Dict[int, str] = {}

        self.pub_detections = rospy.Publisher(
            self.detections_topic,
            Detection2DArray,
            queue_size=10,
        )

        self.pub_legacy_detection = rospy.Publisher(
            self.legacy_detection_topic,
            Detection2D,
            queue_size=10,
        )

        self.pub_detection_image = rospy.Publisher(
            self.detection_image_topic,
            Image,
            queue_size=2,
        )

        self.sub_image = rospy.Subscriber(
            self.image_topic,
            Image,
            self.image_callback,
            queue_size=1,
            buff_size=2 ** 24,
        )

        self._load_model()

        rospy.loginfo("=" * 80)
        rospy.loginfo("[YOLO DETECTOR] Started")
        rospy.loginfo("[YOLO DETECTOR] image_topic: %s", self.image_topic)
        rospy.loginfo("[YOLO DETECTOR] model_path: %s", self.model_path)
        rospy.loginfo("[YOLO DETECTOR] conf_threshold: %.3f", self.conf_threshold)
        rospy.loginfo("[YOLO DETECTOR] iou_threshold: %.3f", self.iou_threshold)
        rospy.loginfo("[YOLO DETECTOR] device: %s", self.device)
        rospy.loginfo("[YOLO DETECTOR] detections_topic: %s", self.detections_topic)
        rospy.loginfo("[YOLO DETECTOR] legacy_detection_topic: %s", self.legacy_detection_topic)
        rospy.loginfo("[YOLO DETECTOR] detection_image_topic: %s", self.detection_image_topic)
        rospy.loginfo("[YOLO DETECTOR] model class names: %s", self.class_names)
        rospy.loginfo("=" * 80)

    @staticmethod
    def _get_package_path_fallback() -> str:
        """
        Fallback path when ROS param does not provide model_path.
        Normally launch file provides the correct path.
        """
        return os.path.abspath(
            os.path.join(
                os.path.dirname(__file__),
                "..",
            )
        )

    def _load_model(self) -> None:
        if YOLO is None:
            rospy.logerr("[YOLO DETECTOR] Could not import ultralytics.")
            rospy.logerr("[YOLO DETECTOR] Import error: %s", str(ULTRALYTICS_IMPORT_ERROR))
            rospy.logerr("[YOLO DETECTOR] Install with:")
            rospy.logerr("python3 -m pip install --user ultralytics")
            raise RuntimeError("ultralytics import failed")

        if not os.path.exists(self.model_path):
            rospy.logerr("[YOLO DETECTOR] Model file not found: %s", self.model_path)
            rospy.logerr("[YOLO DETECTOR] Expected best.pt at models_yolo/best.pt")
            raise FileNotFoundError(self.model_path)

        try:
            self.model = YOLO(self.model_path)
            names = getattr(self.model, "names", {})
            if isinstance(names, dict):
                self.class_names = {int(k): str(v) for k, v in names.items()}
            elif isinstance(names, list):
                self.class_names = {idx: str(v) for idx, v in enumerate(names)}
            else:
                self.class_names = {}
        except Exception as exc:
            rospy.logerr("[YOLO DETECTOR] Failed to load YOLO model.")
            rospy.logerr("[YOLO DETECTOR] model_path: %s", self.model_path)
            rospy.logerr("[YOLO DETECTOR] error: %s", str(exc))
            raise

    def image_callback(self, msg: Image) -> None:
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as exc:
            rospy.logwarn("[YOLO DETECTOR] cv_bridge conversion failed: %s", str(exc))
            return

        try:
            detections, annotated = self.run_inference(cv_image, msg.header)
        except Exception as exc:
            rospy.logerr("[YOLO DETECTOR] Inference failed: %s", str(exc))
            rospy.logdebug(traceback.format_exc())
            return

        array_msg = Detection2DArray()
        array_msg.header = msg.header
        array_msg.detections = detections
        self.pub_detections.publish(array_msg)

        # Legacy compatibility:
        # Publish the best detection as /waste/detection.
        if detections:
            best = max(detections, key=lambda det: det.confidence)
            self.pub_legacy_detection.publish(best)

        if self.publish_annotated and annotated is not None:
            try:
                out_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
                out_msg.header = msg.header
                self.pub_detection_image.publish(out_msg)
            except CvBridgeError as exc:
                rospy.logwarn("[YOLO DETECTOR] annotated image publish failed: %s", str(exc))

    def run_inference(self, image: np.ndarray, header: Header) -> Tuple[List[Detection2D], np.ndarray]:
        if self.model is None:
            return [], image

        results = self.model.predict(
            source=image,
            conf=self.conf_threshold,
            iou=self.iou_threshold,
            device=self.device,
            verbose=False,
        )

        detections: List[Detection2D] = []
        annotated = image.copy()

        if not results:
            return detections, annotated

        result = results[0]
        boxes = getattr(result, "boxes", None)

        if boxes is None or len(boxes) == 0:
            return detections, annotated

        for box in boxes:
            parsed = self._parse_box(box)
            if parsed is None:
                continue

            class_id, raw_class_name, confidence, x_min, y_min, x_max, y_max = parsed
            class_name = normalize_class_name(raw_class_name)

            if class_name is None:
                rospy.logwarn_throttle(
                    2.0,
                    "[YOLO DETECTOR] Ignoring unsupported class from model: %s",
                    raw_class_name,
                )
                continue

            det = Detection2D()
            det.header = header
            det.class_name = class_name
            det.confidence = float(confidence)
            det.x_min = int(x_min)
            det.y_min = int(y_min)
            det.x_max = int(x_max)
            det.y_max = int(y_max)
            det.center_x = int((x_min + x_max) / 2)
            det.center_y = int((y_min + y_max) / 2)

            detections.append(det)

            self._draw_detection(
                annotated,
                class_name=class_name,
                confidence=confidence,
                x_min=x_min,
                y_min=y_min,
                x_max=x_max,
                y_max=y_max,
            )

        detections.sort(key=lambda d: d.confidence, reverse=True)

        return detections, annotated

    def _parse_box(self, box) -> Optional[Tuple[int, str, float, int, int, int, int]]:
        """
        Returns:
          class_id, raw_class_name, confidence, x_min, y_min, x_max, y_max
        """

        try:
            xyxy = box.xyxy[0].detach().cpu().numpy().astype(float)
            conf = float(box.conf[0].detach().cpu().numpy())
            cls_id = int(box.cls[0].detach().cpu().numpy())
        except Exception:
            try:
                xyxy = box.xyxy[0].cpu().numpy().astype(float)
                conf = float(box.conf[0].cpu().numpy())
                cls_id = int(box.cls[0].cpu().numpy())
            except Exception as exc:
                rospy.logwarn("[YOLO DETECTOR] Could not parse box: %s", str(exc))
                return None

        x_min, y_min, x_max, y_max = xyxy.tolist()

        raw_class_name = self.class_names.get(cls_id, str(cls_id))

        return (
            cls_id,
            raw_class_name,
            conf,
            int(round(x_min)),
            int(round(y_min)),
            int(round(x_max)),
            int(round(y_max)),
        )

    @staticmethod
    def _draw_detection(
        image: np.ndarray,
        class_name: str,
        confidence: float,
        x_min: int,
        y_min: int,
        x_max: int,
        y_max: int,
    ) -> None:
        cv2.rectangle(
            image,
            (int(x_min), int(y_min)),
            (int(x_max), int(y_max)),
            (0, 255, 0),
            2,
        )

        label = "{} {:.2f}".format(class_name, confidence)
        cv2.putText(
            image,
            label,
            (int(x_min), max(20, int(y_min) - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )


def main() -> None:
    try:
        node = YoloDetectorNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as exc:
        rospy.logfatal("[YOLO DETECTOR] Fatal error: %s", str(exc))
        rospy.logfatal(traceback.format_exc())
        sys.exit(1)


if __name__ == "__main__":
    main()

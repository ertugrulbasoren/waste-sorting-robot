#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Collect synthetic YOLO dataset from Gazebo realistic 3D waste objects.

Inputs:
  /top_camera/image_raw
  /waste/current_ground_truth

Output:
  datasets/realistic_synthetic_yolo/
    data.yaml
    images/train/*.jpg
    labels/train/*.txt
    images/valid/*.jpg
    labels/valid/*.txt
    debug/*.jpg
    collection_log.csv

Main fix:
  Earlier threshold-based bbox extraction selected large background regions.
  This version first learns a static background image, then uses background
  subtraction to isolate the moving realistic waste object.
"""

import csv
import json
import os
import random
import time
from typing import Dict, Optional, Tuple, List

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String


CLASS_TO_ID = {
    "glass": 0,
    "metal": 1,
    "paper": 2,
    "plastic": 3,
}


def clean(value) -> str:
    return str(value or "").replace("\n", " ").replace("\r", " ").strip()


def project_root_from_script() -> str:
    return os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))


def ensure_dir(path: str) -> None:
    os.makedirs(path, exist_ok=True)


def clamp(value: int, lo: int, hi: int) -> int:
    return max(lo, min(hi, value))


class RealisticSyntheticDatasetCollector:
    def __init__(self):
        self.bridge = CvBridge()

        self.image_topic = clean(rospy.get_param("~image_topic", "/top_camera/image_raw"))
        self.gt_topic = clean(rospy.get_param("~ground_truth_topic", "/waste/current_ground_truth"))

        project_root = project_root_from_script()
        self.output_root = clean(
            rospy.get_param(
                "~output_root",
                os.path.join(project_root, "datasets", "realistic_synthetic_yolo"),
            )
        )

        self.max_images = int(rospy.get_param("~max_images", 1000))
        self.images_per_object = int(rospy.get_param("~images_per_object", 4))
        self.capture_interval_sec = float(rospy.get_param("~capture_interval_sec", 0.35))
        self.valid_ratio = float(rospy.get_param("~valid_ratio", 0.20))

        self.background_frames_required = int(rospy.get_param("~background_frames_required", 20))
        self.background_diff_threshold = int(rospy.get_param("~background_diff_threshold", 22))

        self.min_bbox_area = int(rospy.get_param("~min_bbox_area", 300))
        self.max_bbox_area_ratio = float(rospy.get_param("~max_bbox_area_ratio", 0.25))

        self.min_bbox_width = int(rospy.get_param("~min_bbox_width", 12))
        self.min_bbox_height = int(rospy.get_param("~min_bbox_height", 12))

        self.roi_x_min_ratio = float(rospy.get_param("~roi_x_min_ratio", 0.0))
        self.roi_y_min_ratio = float(rospy.get_param("~roi_y_min_ratio", 0.0))
        self.roi_x_max_ratio = float(rospy.get_param("~roi_x_max_ratio", 1.0))
        self.roi_y_max_ratio = float(rospy.get_param("~roi_y_max_ratio", 1.0))

        self.debug = bool(rospy.get_param("~debug", True))

        self.background_frames: List[np.ndarray] = []
        self.background_gray: Optional[np.ndarray] = None
        self.background_ready = False

        self.current_gt: Optional[Dict] = None
        self.current_model_name: str = ""
        self.current_class_name: str = ""
        self.current_asset_name: str = ""
        self.current_sequence_id: int = -1

        self.captures_for_current_model = 0
        self.total_saved = 0
        self.total_skipped = 0
        self.last_capture_time = 0.0

        self.class_counts = {name: 0 for name in CLASS_TO_ID.keys()}

        self.prepare_output_dirs()
        self.write_data_yaml()
        self.ensure_log_file()

        rospy.Subscriber(self.gt_topic, String, self.gt_callback, queue_size=10)
        rospy.Subscriber(self.image_topic, Image, self.image_callback, queue_size=1)

        rospy.loginfo("=" * 90)
        rospy.loginfo("[SYNTHETIC COLLECTOR] Started - BACKGROUND SUBTRACTION MODE")
        rospy.loginfo("[SYNTHETIC COLLECTOR] image_topic: %s", self.image_topic)
        rospy.loginfo("[SYNTHETIC COLLECTOR] ground_truth_topic: %s", self.gt_topic)
        rospy.loginfo("[SYNTHETIC COLLECTOR] output_root: %s", self.output_root)
        rospy.loginfo("[SYNTHETIC COLLECTOR] max_images: %d", self.max_images)
        rospy.loginfo("[SYNTHETIC COLLECTOR] images_per_object: %d", self.images_per_object)
        rospy.loginfo("[SYNTHETIC COLLECTOR] background_frames_required: %d", self.background_frames_required)
        rospy.loginfo("=" * 90)

    def prepare_output_dirs(self):
        for split in ["train", "valid"]:
            ensure_dir(os.path.join(self.output_root, "images", split))
            ensure_dir(os.path.join(self.output_root, "labels", split))

        ensure_dir(os.path.join(self.output_root, "debug"))

    def write_data_yaml(self):
        path = os.path.join(self.output_root, "data.yaml")

        content = """train: images/train
val: images/valid

nc: 4

names:
  0: glass
  1: metal
  2: paper
  3: plastic
"""

        with open(path, "w") as f:
            f.write(content)

    def ensure_log_file(self):
        path = os.path.join(self.output_root, "collection_log.csv")

        if os.path.exists(path):
            return

        with open(path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(
                [
                    "wall_time",
                    "image_name",
                    "split",
                    "class_name",
                    "class_id",
                    "model_name",
                    "asset_name",
                    "sequence_id",
                    "bbox_x_min",
                    "bbox_y_min",
                    "bbox_x_max",
                    "bbox_y_max",
                    "bbox_area",
                    "saved",
                    "reason",
                ]
            )

    def append_log(
        self,
        image_name: str,
        split: str,
        class_name: str,
        class_id: int,
        model_name: str,
        asset_name: str,
        sequence_id: int,
        bbox: Optional[Tuple[int, int, int, int]],
        saved: bool,
        reason: str,
    ):
        path = os.path.join(self.output_root, "collection_log.csv")

        if bbox is None:
            x_min = y_min = x_max = y_max = area = 0
        else:
            x_min, y_min, x_max, y_max = bbox
            area = max(0, x_max - x_min) * max(0, y_max - y_min)

        with open(path, "a", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(
                [
                    round(time.time(), 3),
                    image_name,
                    split,
                    class_name,
                    class_id,
                    model_name,
                    asset_name,
                    sequence_id,
                    x_min,
                    y_min,
                    x_max,
                    y_max,
                    area,
                    saved,
                    reason,
                ]
            )

    def gt_callback(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except Exception:
            rospy.logwarn("[SYNTHETIC COLLECTOR] Invalid GT JSON")
            return

        model_name = clean(payload.get("model_name", ""))
        class_name = clean(payload.get("class_name", ""))
        asset_name = clean(payload.get("asset_name", ""))
        sequence_id = int(payload.get("sequence_id", -1))

        if class_name not in CLASS_TO_ID:
            return

        if model_name != self.current_model_name:
            self.captures_for_current_model = 0

        self.current_gt = payload
        self.current_model_name = model_name
        self.current_class_name = class_name
        self.current_asset_name = asset_name
        self.current_sequence_id = sequence_id

        rospy.loginfo(
            "[SYNTHETIC COLLECTOR] Current GT model=%s class=%s asset=%s seq=%s",
            model_name,
            class_name,
            asset_name,
            str(sequence_id),
        )

    def update_background(self, image_bgr: np.ndarray) -> None:
        gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)

        self.background_frames.append(gray)

        rospy.loginfo(
            "[SYNTHETIC COLLECTOR] Learning background frame %d/%d",
            len(self.background_frames),
            self.background_frames_required,
        )

        if len(self.background_frames) >= self.background_frames_required:
            stack = np.stack(self.background_frames, axis=0)
            self.background_gray = np.median(stack, axis=0).astype(np.uint8)
            self.background_ready = True
            rospy.loginfo("[SYNTHETIC COLLECTOR] Background model is ready.")

    def estimate_bbox(self, image_bgr: np.ndarray) -> Optional[Tuple[int, int, int, int]]:
        if self.background_gray is None:
            return None

        h, w = image_bgr.shape[:2]

        rx1 = int(w * self.roi_x_min_ratio)
        ry1 = int(h * self.roi_y_min_ratio)
        rx2 = int(w * self.roi_x_max_ratio)
        ry2 = int(h * self.roi_y_max_ratio)

        rx1 = clamp(rx1, 0, w - 1)
        ry1 = clamp(ry1, 0, h - 1)
        rx2 = clamp(rx2, rx1 + 1, w)
        ry2 = clamp(ry2, ry1 + 1, h)

        gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)

        bg_roi = self.background_gray[ry1:ry2, rx1:rx2]
        gray_roi = gray[ry1:ry2, rx1:rx2]

        diff = cv2.absdiff(gray_roi, bg_roi)

        _, mask = cv2.threshold(
            diff,
            self.background_diff_threshold,
            255,
            cv2.THRESH_BINARY,
        )

        kernel3 = np.ones((3, 3), np.uint8)
        kernel5 = np.ones((5, 5), np.uint8)

        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel3, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel5, iterations=2)
        mask = cv2.dilate(mask, kernel3, iterations=1)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            return None

        image_area = w * h
        candidates = []

        for contour in contours:
            x, y, bw, bh = cv2.boundingRect(contour)
            area = bw * bh

            if area < self.min_bbox_area:
                continue

            if area > image_area * self.max_bbox_area_ratio:
                continue

            if bw < self.min_bbox_width or bh < self.min_bbox_height:
                continue

            ratio = bw / float(max(1, bh))

            if ratio > 8.0 or ratio < 0.10:
                continue

            candidates.append((area, x, y, bw, bh))

        if not candidates:
            return None

        candidates.sort(reverse=True, key=lambda item: item[0])
        _, x, y, bw, bh = candidates[0]

        pad = 8

        x_min = clamp(rx1 + x - pad, 0, w - 1)
        y_min = clamp(ry1 + y - pad, 0, h - 1)
        x_max = clamp(rx1 + x + bw + pad, 0, w - 1)
        y_max = clamp(ry1 + y + bh + pad, 0, h - 1)

        if x_max <= x_min or y_max <= y_min:
            return None

        return x_min, y_min, x_max, y_max

    def save_sample(self, image_bgr: np.ndarray, bbox: Tuple[int, int, int, int]):
        h, w = image_bgr.shape[:2]

        class_name = self.current_class_name
        class_id = CLASS_TO_ID[class_name]
        model_name = self.current_model_name
        asset_name = self.current_asset_name
        sequence_id = self.current_sequence_id

        split = "valid" if random.random() < self.valid_ratio else "train"

        self.class_counts[class_name] += 1
        self.total_saved += 1
        self.captures_for_current_model += 1

        image_name = "{:06d}_{}_{}.jpg".format(
            self.total_saved,
            class_name,
            clean(model_name).replace("/", "_"),
        )

        label_name = image_name.replace(".jpg", ".txt")

        image_path = os.path.join(self.output_root, "images", split, image_name)
        label_path = os.path.join(self.output_root, "labels", split, label_name)

        x_min, y_min, x_max, y_max = bbox

        cx = ((x_min + x_max) / 2.0) / w
        cy = ((y_min + y_max) / 2.0) / h
        bw = (x_max - x_min) / float(w)
        bh = (y_max - y_min) / float(h)

        cv2.imwrite(image_path, image_bgr)

        with open(label_path, "w") as f:
            f.write("{} {:.6f} {:.6f} {:.6f} {:.6f}\n".format(class_id, cx, cy, bw, bh))

        if self.debug:
            debug_img = image_bgr.copy()
            cv2.rectangle(debug_img, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
            cv2.putText(
                debug_img,
                "{} {}".format(class_name, model_name),
                (max(0, x_min), max(20, y_min - 5)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 255, 0),
                2,
            )
            debug_path = os.path.join(self.output_root, "debug", image_name)
            cv2.imwrite(debug_path, debug_img)

        self.append_log(
            image_name=image_name,
            split=split,
            class_name=class_name,
            class_id=class_id,
            model_name=model_name,
            asset_name=asset_name,
            sequence_id=sequence_id,
            bbox=bbox,
            saved=True,
            reason="saved",
        )

        rospy.loginfo(
            "[SYNTHETIC COLLECTOR] Saved %s class=%s bbox=%s total=%d/%d",
            image_name,
            class_name,
            str(bbox),
            self.total_saved,
            self.max_images,
        )

    def image_callback(self, msg: Image):
        if self.total_saved >= self.max_images:
            return

        try:
            image_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            rospy.logwarn("[SYNTHETIC COLLECTOR] cv_bridge failed: %s", str(exc))
            return

        if not self.background_ready:
            self.update_background(image_bgr)
            return

        if self.current_gt is None:
            return

        if not self.current_model_name or self.current_class_name not in CLASS_TO_ID:
            return

        if self.captures_for_current_model >= self.images_per_object:
            return

        now = time.time()

        if now - self.last_capture_time < self.capture_interval_sec:
            return

        bbox = self.estimate_bbox(image_bgr)

        if bbox is None:
            self.total_skipped += 1
            self.append_log(
                image_name="",
                split="",
                class_name=self.current_class_name,
                class_id=CLASS_TO_ID[self.current_class_name],
                model_name=self.current_model_name,
                asset_name=self.current_asset_name,
                sequence_id=self.current_sequence_id,
                bbox=None,
                saved=False,
                reason="bbox_not_found",
            )
            return

        self.last_capture_time = now
        self.save_sample(image_bgr, bbox)

        if self.total_saved >= self.max_images:
            rospy.loginfo("=" * 90)
            rospy.loginfo("[SYNTHETIC COLLECTOR] Collection target reached.")
            rospy.loginfo("[SYNTHETIC COLLECTOR] total_saved: %d", self.total_saved)
            rospy.loginfo("[SYNTHETIC COLLECTOR] total_skipped: %d", self.total_skipped)
            rospy.loginfo("[SYNTHETIC COLLECTOR] class_counts: %s", str(self.class_counts))
            rospy.loginfo("[SYNTHETIC COLLECTOR] output_root: %s", self.output_root)
            rospy.loginfo("=" * 90)


def main():
    rospy.init_node("realistic_synthetic_dataset_collector", anonymous=False)
    RealisticSyntheticDatasetCollector()
    rospy.spin()


if __name__ == "__main__":
    main()

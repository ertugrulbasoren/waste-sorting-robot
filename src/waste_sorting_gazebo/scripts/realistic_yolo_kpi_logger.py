#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Realistic YOLO KPI Logger.

This node evaluates the full realistic 3D YOLO pipeline.

It listens to:
  /waste/current_ground_truth
  /waste/pick_event_resolved

It writes:
  logs/trials/realistic_yolo_kpi.csv

Purpose:
  Measure whether YOLO predicted class matches the active realistic feeder
  ground truth class.

Pipeline evaluated:
  realistic_conveyor_feeder
      ↓
  top camera
      ↓
  YOLO detector
      ↓
  tracker
      ↓
  pick_event_model_resolver
      ↓
  realistic_yolo_kpi_logger

Important:
  This logger does not control sorting.
  It only records KPI rows.
"""

import csv
import json
import os
import time
from typing import Dict, Optional, Set

import rospy
from std_msgs.msg import String


VALID_CLASSES = ["glass", "metal", "paper", "plastic"]


def clean(value) -> str:
    return str(value or "").replace("\n", " ").replace("\r", " ").strip()


def project_root_from_script() -> str:
    return os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))


class RealisticYoloKpiLogger:
    def __init__(self):
        project_root = project_root_from_script()

        self.gt_topic = clean(rospy.get_param("~ground_truth_topic", "/waste/current_ground_truth"))
        self.pick_event_topic = clean(rospy.get_param("~pick_event_topic", "/waste/pick_event_resolved"))

        self.log_path = clean(
            rospy.get_param(
                "~log_path",
                os.path.join(project_root, "logs", "trials", "realistic_yolo_kpi.csv"),
            )
        )

        self.max_gt_age_sec = float(rospy.get_param("~max_gt_age_sec", 20.0))
        self.ignore_duplicate_track_ids = bool(rospy.get_param("~ignore_duplicate_track_ids", True))
        self.ignore_duplicate_model_names = bool(rospy.get_param("~ignore_duplicate_model_names", True))

        self.current_gt: Optional[Dict] = None
        self.current_gt_wall_time: float = 0.0

        self.logged_track_ids: Set[int] = set()
        self.logged_model_names: Set[str] = set()

        self.total_events = 0
        self.correct_events = 0

        self.ensure_log_file()

        rospy.Subscriber(self.gt_topic, String, self.gt_callback, queue_size=10)
        rospy.Subscriber(self.pick_event_topic, String, self.pick_event_callback, queue_size=50)

        rospy.loginfo("=" * 90)
        rospy.loginfo("[REALISTIC YOLO KPI LOGGER] Started")
        rospy.loginfo("[REALISTIC YOLO KPI LOGGER] ground_truth_topic: %s", self.gt_topic)
        rospy.loginfo("[REALISTIC YOLO KPI LOGGER] pick_event_topic: %s", self.pick_event_topic)
        rospy.loginfo("[REALISTIC YOLO KPI LOGGER] log_path: %s", self.log_path)
        rospy.loginfo("[REALISTIC YOLO KPI LOGGER] max_gt_age_sec: %.2f", self.max_gt_age_sec)
        rospy.loginfo("=" * 90)

    def ensure_log_file(self):
        os.makedirs(os.path.dirname(self.log_path), exist_ok=True)

        if os.path.exists(self.log_path):
            return

        with open(self.log_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(
                [
                    "wall_time",
                    "ros_stamp",
                    "model_name",
                    "gt_class",
                    "pred_class",
                    "confidence",
                    "correct",
                    "track_id",
                    "center_x",
                    "center_y",
                    "resolver_used",
                    "resolver_gt_class",
                    "resolver_gt_age_sec",
                    "event_type",
                    "state",
                ]
            )

    def gt_callback(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except Exception:
            rospy.logwarn("[REALISTIC YOLO KPI LOGGER] Invalid GT JSON")
            return

        model_name = clean(payload.get("model_name", ""))
        class_name = clean(payload.get("class_name", ""))

        if not model_name or class_name not in VALID_CLASSES:
            return

        self.current_gt = payload
        self.current_gt_wall_time = time.time()

        rospy.loginfo(
            "[REALISTIC YOLO KPI LOGGER] Current GT model=%s class=%s",
            model_name,
            class_name,
        )

    def pick_event_callback(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except Exception:
            rospy.logwarn("[REALISTIC YOLO KPI LOGGER] Invalid pick_event JSON")
            return

        model_name = clean(payload.get("model_name", ""))
        pred_class = clean(payload.get("class_name", ""))

        if pred_class not in VALID_CLASSES:
            rospy.logwarn("[REALISTIC YOLO KPI LOGGER] Invalid predicted class: %s", pred_class)
            return

        track_id_raw = payload.get("track_id", -1)

        try:
            track_id = int(track_id_raw)
        except Exception:
            track_id = -1

        if self.ignore_duplicate_track_ids and track_id in self.logged_track_ids:
            rospy.loginfo("[REALISTIC YOLO KPI LOGGER] Duplicate track_id ignored: %s", str(track_id))
            return

        if self.ignore_duplicate_model_names and model_name and model_name in self.logged_model_names:
            rospy.loginfo("[REALISTIC YOLO KPI LOGGER] Duplicate model_name ignored: %s", model_name)
            return

        if self.current_gt is None:
            rospy.logwarn("[REALISTIC YOLO KPI LOGGER] No GT available. Event ignored.")
            return

        gt_age = time.time() - self.current_gt_wall_time

        if gt_age > self.max_gt_age_sec:
            rospy.logwarn(
                "[REALISTIC YOLO KPI LOGGER] GT too old %.2f sec. Event ignored.",
                gt_age,
            )
            return

        gt_class = clean(self.current_gt.get("class_name", ""))

        resolver_info = payload.get("resolved_from_ground_truth", {})
        resolver_gt_class = clean(resolver_info.get("gt_class_name", ""))
        resolver_gt_age_sec = resolver_info.get("gt_age_sec", "")

        # Prefer resolver GT if present because it is tied to the resolved model_name.
        if resolver_gt_class in VALID_CLASSES:
            gt_class_for_eval = resolver_gt_class
        else:
            gt_class_for_eval = gt_class

        correct = pred_class == gt_class_for_eval

        confidence = float(payload.get("confidence", 0.0))
        center_x = int(payload.get("center_x", -1))
        center_y = int(payload.get("center_y", -1))
        ros_stamp = payload.get("stamp", "")
        resolver_used = bool(payload.get("resolver_used", False))
        event_type = clean(payload.get("event_type", ""))
        state = clean(payload.get("state", ""))

        self.append_row(
            ros_stamp=ros_stamp,
            model_name=model_name,
            gt_class=gt_class_for_eval,
            pred_class=pred_class,
            confidence=confidence,
            correct=correct,
            track_id=track_id,
            center_x=center_x,
            center_y=center_y,
            resolver_used=resolver_used,
            resolver_gt_class=resolver_gt_class,
            resolver_gt_age_sec=resolver_gt_age_sec,
            event_type=event_type,
            state=state,
        )

        if track_id >= 0:
            self.logged_track_ids.add(track_id)

        if model_name:
            self.logged_model_names.add(model_name)

        self.total_events += 1

        if correct:
            self.correct_events += 1

        acc = (self.correct_events / self.total_events * 100.0) if self.total_events else 0.0

        rospy.loginfo(
            "[REALISTIC YOLO KPI LOGGER] Logged model=%s gt=%s pred=%s conf=%.3f correct=%s running_acc=%.2f%% (%d/%d)",
            model_name,
            gt_class_for_eval,
            pred_class,
            confidence,
            str(correct),
            acc,
            self.correct_events,
            self.total_events,
        )

    def append_row(
        self,
        ros_stamp,
        model_name: str,
        gt_class: str,
        pred_class: str,
        confidence: float,
        correct: bool,
        track_id: int,
        center_x: int,
        center_y: int,
        resolver_used: bool,
        resolver_gt_class: str,
        resolver_gt_age_sec,
        event_type: str,
        state: str,
    ):
        with open(self.log_path, "a", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(
                [
                    round(time.time(), 3),
                    ros_stamp,
                    model_name,
                    gt_class,
                    pred_class,
                    round(confidence, 6),
                    correct,
                    track_id,
                    center_x,
                    center_y,
                    resolver_used,
                    resolver_gt_class,
                    resolver_gt_age_sec,
                    event_type,
                    state,
                ]
            )


def main():
    rospy.init_node("realistic_yolo_kpi_logger", anonymous=False)
    RealisticYoloKpiLogger()
    rospy.spin()


if __name__ == "__main__":
    main()

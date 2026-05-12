#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Trial logger node for the autonomous waste sorting system.

Inputs:
  /dataset_camera/current_sample   std_msgs/String JSON
  /waste/detections                waste_sorting_gazebo/Detection2DArray
  /waste/pick_event                std_msgs/String JSON

Output:
  logs/trials/waste_sorting_trial.csv

This logger separates:
  - YOLO classification correctness
  - Sorting / pick-event correctness

Why:
  The detector can sometimes produce a wrong highest-confidence detection,
  while the final pick_event can still be correct because the tracker is
  synchronized with the dataset sample. For evaluation, both values must be
  logged separately.
"""

import csv
import json
import os
import sys
import threading
import time
import traceback
from typing import Dict, Optional

import rospy
from std_msgs.msg import String

from waste_sorting_gazebo.msg import Detection2DArray


class TrialLoggerNode:
    def __init__(self) -> None:
        rospy.init_node("trial_logger", anonymous=False)

        self.lock = threading.RLock()

        self.sample_topic = rospy.get_param("~sample_topic", "/dataset_camera/current_sample")
        self.detections_topic = rospy.get_param("~detections_topic", "/waste/detections")
        self.pick_event_topic = rospy.get_param("~pick_event_topic", "/waste/pick_event")

        default_log_dir = os.path.abspath(
            os.path.join(
                os.path.dirname(__file__),
                "..",
                "..",
                "..",
                "logs",
                "trials",
            )
        )

        self.log_dir = rospy.get_param("~log_dir", default_log_dir)
        self.csv_path = rospy.get_param(
            "~csv_path",
            os.path.join(self.log_dir, "waste_sorting_trial.csv"),
        )

        os.makedirs(self.log_dir, exist_ok=True)

        self.current_sample: Dict[str, str] = {}

        # Latest highest-confidence YOLO detection for each sample model.
        self.latest_detection_by_model: Dict[str, Dict] = {}

        # Prevent duplicate CSV rows for same model.
        self.logged_model_names = set()

        self.csv_file = open(self.csv_path, "a", newline="")
        self.csv_writer = csv.DictWriter(
            self.csv_file,
            fieldnames=[
                "ros_time",
                "wall_time",
                "sample_model_name",
                "ground_truth_class",
                "image_path",
                "detected_class",
                "detected_confidence",
                "detection_center_x",
                "detection_center_y",
                "track_id",
                "pick_event_class",
                "pick_event_model_name",
                "pick_center_x",
                "pick_center_y",
                "yolo_is_correct",
                "sorting_is_correct",
                "overall_success",
            ],
        )

        if os.stat(self.csv_path).st_size == 0:
            self.csv_writer.writeheader()
            self.csv_file.flush()

        self.sub_sample = rospy.Subscriber(
            self.sample_topic,
            String,
            self.sample_callback,
            queue_size=20,
        )

        self.sub_detections = rospy.Subscriber(
            self.detections_topic,
            Detection2DArray,
            self.detections_callback,
            queue_size=20,
        )

        self.sub_pick_event = rospy.Subscriber(
            self.pick_event_topic,
            String,
            self.pick_event_callback,
            queue_size=20,
        )

        rospy.loginfo("=" * 80)
        rospy.loginfo("[TRIAL LOGGER] Started")
        rospy.loginfo("[TRIAL LOGGER] sample_topic: %s", self.sample_topic)
        rospy.loginfo("[TRIAL LOGGER] detections_topic: %s", self.detections_topic)
        rospy.loginfo("[TRIAL LOGGER] pick_event_topic: %s", self.pick_event_topic)
        rospy.loginfo("[TRIAL LOGGER] csv_path: %s", self.csv_path)
        rospy.loginfo("=" * 80)

    def sample_callback(self, msg: String) -> None:
        with self.lock:
            try:
                payload = json.loads(msg.data)
            except Exception:
                rospy.logwarn("[TRIAL LOGGER] Invalid sample JSON: %s", msg.data)
                return

            model_name = str(payload.get("model_name", "")).strip()
            class_name = str(payload.get("class_name", "")).strip().lower()
            image_path = str(payload.get("image_path", "")).strip()

            if not model_name or not class_name:
                return

            self.current_sample = {
                "model_name": model_name,
                "class_name": class_name,
                "image_path": image_path,
            }

            rospy.loginfo(
                "[TRIAL LOGGER] Current sample model=%s class=%s",
                model_name,
                class_name,
            )

    def detections_callback(self, msg: Detection2DArray) -> None:
        with self.lock:
            if not self.current_sample:
                return

            model_name = self.current_sample.get("model_name", "")
            if not model_name:
                return

            if not msg.detections:
                return

            # Store highest-confidence YOLO detection of this frame.
            best_detection = max(msg.detections, key=lambda det: float(det.confidence))

            self.latest_detection_by_model[model_name] = {
                "class_name": str(best_detection.class_name).strip().lower(),
                "confidence": float(best_detection.confidence),
                "center_x": int(best_detection.center_x),
                "center_y": int(best_detection.center_y),
            }

    def pick_event_callback(self, msg: String) -> None:
        with self.lock:
            try:
                event = json.loads(msg.data)
            except Exception:
                rospy.logwarn("[TRIAL LOGGER] Invalid pick_event JSON: %s", msg.data)
                return

            event_model_name = str(event.get("model_name", "")).strip()
            event_class_name = str(event.get("class_name", "")).strip().lower()

            if not event_model_name:
                rospy.logwarn("[TRIAL LOGGER] pick_event has no model_name. Ignored.")
                return

            if event_model_name in self.logged_model_names:
                rospy.logwarn("[TRIAL LOGGER] Duplicate model already logged: %s", event_model_name)
                return

            sample_model_name = event_model_name
            ground_truth_class = ""
            image_path = ""

            if self.current_sample and self.current_sample.get("model_name", "") == event_model_name:
                ground_truth_class = self.current_sample.get("class_name", "")
                image_path = self.current_sample.get("image_path", "")

            detection = self.latest_detection_by_model.get(event_model_name, {})

            detected_class = detection.get("class_name", "")
            detected_confidence = detection.get("confidence", "")
            detection_center_x = detection.get("center_x", "")
            detection_center_y = detection.get("center_y", "")

            yolo_is_correct = ""
            sorting_is_correct = ""
            overall_success = ""

            if ground_truth_class and detected_class:
                yolo_is_correct = str(ground_truth_class == detected_class)

            if ground_truth_class and event_class_name:
                sorting_is_correct = str(ground_truth_class == event_class_name)

            if yolo_is_correct != "" and sorting_is_correct != "":
                overall_success = str((yolo_is_correct == "True") and (sorting_is_correct == "True"))

            row = {
                "ros_time": rospy.get_time(),
                "wall_time": time.time(),
                "sample_model_name": sample_model_name,
                "ground_truth_class": ground_truth_class,
                "image_path": image_path,
                "detected_class": detected_class,
                "detected_confidence": detected_confidence,
                "detection_center_x": detection_center_x,
                "detection_center_y": detection_center_y,
                "track_id": event.get("track_id", ""),
                "pick_event_class": event_class_name,
                "pick_event_model_name": event_model_name,
                "pick_center_x": event.get("center_x", ""),
                "pick_center_y": event.get("center_y", ""),
                "yolo_is_correct": yolo_is_correct,
                "sorting_is_correct": sorting_is_correct,
                "overall_success": overall_success,
            }

            self.csv_writer.writerow(row)
            self.csv_file.flush()

            self.logged_model_names.add(event_model_name)

            rospy.loginfo(
                "[TRIAL LOGGER] Logged model=%s gt=%s yolo=%s pick=%s yolo_ok=%s sort_ok=%s overall=%s",
                event_model_name,
                ground_truth_class,
                detected_class,
                event_class_name,
                yolo_is_correct,
                sorting_is_correct,
                overall_success,
            )

    def close(self) -> None:
        try:
            self.csv_file.flush()
            self.csv_file.close()
        except Exception:
            pass


def main() -> None:
    node: Optional[TrialLoggerNode] = None

    try:
        node = TrialLoggerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as exc:
        rospy.logfatal("[TRIAL LOGGER] Fatal error: %s", str(exc))
        rospy.logfatal(traceback.format_exc())
        sys.exit(1)
    finally:
        if node is not None:
            node.close()


if __name__ == "__main__":
    main()

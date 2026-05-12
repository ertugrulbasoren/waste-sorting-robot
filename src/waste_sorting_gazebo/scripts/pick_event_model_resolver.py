#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Pick Event Model Resolver.

Purpose:
  Tracker publishes /waste/pick_event based on YOLO detections.
  In realistic feeder mode, tracker usually does not know the active Gazebo
  model_name, so model_name can be empty.

This node listens to:
  /waste/current_ground_truth
  /waste/pick_event

It publishes:
  /waste/pick_event_resolved

Important rules:
  1. YOLO/tracker class_name is NOT corrected.
  2. Resolver only fills model_name from the active feeder ground truth.
  3. Very early events after a new object spawn are ignored.
     This prevents stale/unstable tracks from the previous object being
     attached to the new object.
"""

import json
import time
from typing import Dict, Optional

import rospy
from std_msgs.msg import String


VALID_CLASSES = ["glass", "metal", "paper", "plastic"]


def clean(value) -> str:
    return str(value or "").replace("\n", " ").replace("\r", " ").strip()


class PickEventModelResolver:
    def __init__(self):
        self.gt_topic = clean(rospy.get_param("~ground_truth_topic", "/waste/current_ground_truth"))
        self.input_topic = clean(rospy.get_param("~input_pick_event_topic", "/waste/pick_event"))
        self.output_topic = clean(rospy.get_param("~output_pick_event_topic", "/waste/pick_event_resolved"))

        self.max_gt_age_sec = float(rospy.get_param("~max_gt_age_sec", 15.0))

        # Critical filter:
        # Ignore pick_events that arrive too soon after a new object was spawned.
        # This reduces stale tracker events and unstable early predictions.
        self.min_gt_age_sec = float(rospy.get_param("~min_gt_age_sec", 3.0))

        self.current_gt: Optional[Dict] = None
        self.current_gt_wall_time: float = 0.0
        self.current_model_name: str = ""
        self.current_gt_class: str = ""

        self.published_track_ids = set()
        self.published_model_names = set()

        self.ignore_duplicate_track_ids = bool(rospy.get_param("~ignore_duplicate_track_ids", True))
        self.ignore_duplicate_model_names = bool(rospy.get_param("~ignore_duplicate_model_names", True))

        self.pub = rospy.Publisher(self.output_topic, String, queue_size=20)

        rospy.Subscriber(self.gt_topic, String, self.gt_callback, queue_size=10)
        rospy.Subscriber(self.input_topic, String, self.pick_callback, queue_size=50)

        rospy.loginfo("=" * 90)
        rospy.loginfo("[PICK EVENT RESOLVER] Started")
        rospy.loginfo("[PICK EVENT RESOLVER] ground_truth_topic: %s", self.gt_topic)
        rospy.loginfo("[PICK EVENT RESOLVER] input_pick_event_topic: %s", self.input_topic)
        rospy.loginfo("[PICK EVENT RESOLVER] output_pick_event_topic: %s", self.output_topic)
        rospy.loginfo("[PICK EVENT RESOLVER] min_gt_age_sec: %.2f", self.min_gt_age_sec)
        rospy.loginfo("[PICK EVENT RESOLVER] max_gt_age_sec: %.2f", self.max_gt_age_sec)
        rospy.loginfo("=" * 90)

    def gt_callback(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except Exception:
            rospy.logwarn("[PICK EVENT RESOLVER] Invalid ground truth JSON")
            return

        model_name = clean(payload.get("model_name", ""))
        class_name = clean(payload.get("class_name", ""))

        if not model_name:
            return

        self.current_gt = payload
        self.current_gt_wall_time = time.time()
        self.current_model_name = model_name
        self.current_gt_class = class_name

        rospy.loginfo(
            "[PICK EVENT RESOLVER] Current GT model=%s class=%s",
            model_name,
            class_name,
        )

    def pick_callback(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except Exception:
            rospy.logwarn("[PICK EVENT RESOLVER] Invalid pick_event JSON")
            return

        track_id = self.safe_int(payload.get("track_id", -1))
        original_model_name = clean(payload.get("model_name", ""))
        pred_class = clean(payload.get("class_name", ""))

        if pred_class not in VALID_CLASSES:
            rospy.logwarn("[PICK EVENT RESOLVER] Invalid YOLO class ignored: %s", pred_class)
            return

        if self.ignore_duplicate_track_ids and track_id in self.published_track_ids:
            rospy.loginfo("[PICK EVENT RESOLVER] Duplicate track_id ignored: %s", str(track_id))
            return

        if original_model_name and self.ignore_duplicate_model_names and original_model_name in self.published_model_names:
            rospy.loginfo("[PICK EVENT RESOLVER] Duplicate model_name ignored: %s", original_model_name)
            return

        if self.current_gt is None:
            rospy.logwarn("[PICK EVENT RESOLVER] No current GT. Cannot resolve model_name.")
            return

        age = time.time() - self.current_gt_wall_time

        if age < self.min_gt_age_sec:
            rospy.logwarn(
                "[PICK EVENT RESOLVER] Event ignored because GT age is too young: %.2f sec < %.2f sec. track_id=%s class=%s",
                age,
                self.min_gt_age_sec,
                str(track_id),
                pred_class,
            )
            return

        if age > self.max_gt_age_sec:
            rospy.logwarn(
                "[PICK EVENT RESOLVER] Event ignored because GT is too old: %.2f sec > %.2f sec. track_id=%s class=%s",
                age,
                self.max_gt_age_sec,
                str(track_id),
                pred_class,
            )
            return

        resolved_model_name = clean(self.current_gt.get("model_name", ""))
        gt_class_name = clean(self.current_gt.get("class_name", ""))

        if not resolved_model_name:
            rospy.logwarn("[PICK EVENT RESOLVER] GT has empty model_name. Event ignored.")
            return

        # Keep YOLO/tracker class_name.
        # Only fill model_name.
        payload["model_name"] = resolved_model_name
        payload["resolver_used"] = True
        payload["resolved_from_ground_truth"] = {
            "model_name": resolved_model_name,
            "gt_class_name": gt_class_name,
            "gt_age_sec": round(age, 3),
        }

        self.published_track_ids.add(track_id)
        self.published_model_names.add(resolved_model_name)

        rospy.loginfo(
            "[PICK EVENT RESOLVER] Resolved pick_event track_id=%s yolo_class=%s model=%s gt_class=%s gt_age=%.2f",
            str(track_id),
            pred_class,
            resolved_model_name,
            gt_class_name,
            age,
        )

        self.publish(payload)

    def publish(self, payload: Dict):
        msg = String()
        msg.data = json.dumps(payload, sort_keys=True)
        self.pub.publish(msg)

    @staticmethod
    def safe_int(value) -> int:
        try:
            return int(value)
        except Exception:
            return -1


def main():
    rospy.init_node("pick_event_model_resolver", anonymous=False)
    PickEventModelResolver()
    rospy.spin()


if __name__ == "__main__":
    main()

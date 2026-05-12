#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Resolve empty model_name in tracker pick_event messages.

Problem:
  tracker_node.py publishes /waste/pick_event based on YOLO detections.
  In the realistic 3D feeder mode, tracker does not know the active Gazebo
  model name, so pick_event may contain:

    "model_name": ""

  sort_executor needs a Gazebo model_name to move the object reliably.

Solution:
  This node listens to:
    /waste/current_ground_truth
    /waste/pick_event

  It republishes:
    /waste/pick_event_resolved

  The class_name is kept from YOLO/tracker.
  The model_name is filled from the current active realistic feeder object.

Important:
  This node does NOT correct YOLO class.
  If YOLO says "metal", output class_name remains "metal".
  It only attaches the currently active Gazebo model_name.
"""

import json
import time
from typing import Optional, Dict

import rospy
from std_msgs.msg import String


def clean(value) -> str:
    return str(value or "").replace("\n", " ").replace("\r", " ").strip()


class PickEventModelResolver:
    def __init__(self):
        self.gt_topic = clean(rospy.get_param("~ground_truth_topic", "/waste/current_ground_truth"))
        self.input_topic = clean(rospy.get_param("~input_pick_event_topic", "/waste/pick_event"))
        self.output_topic = clean(rospy.get_param("~output_pick_event_topic", "/waste/pick_event_resolved"))

        self.max_gt_age_sec = float(rospy.get_param("~max_gt_age_sec", 15.0))

        self.current_gt: Optional[Dict] = None
        self.current_gt_wall_time: float = 0.0

        self.pub = rospy.Publisher(self.output_topic, String, queue_size=20)

        rospy.Subscriber(self.gt_topic, String, self.gt_callback, queue_size=10)
        rospy.Subscriber(self.input_topic, String, self.pick_callback, queue_size=50)

        rospy.loginfo("=" * 90)
        rospy.loginfo("[PICK EVENT RESOLVER] Started")
        rospy.loginfo("[PICK EVENT RESOLVER] ground_truth_topic: %s", self.gt_topic)
        rospy.loginfo("[PICK EVENT RESOLVER] input_pick_event_topic: %s", self.input_topic)
        rospy.loginfo("[PICK EVENT RESOLVER] output_pick_event_topic: %s", self.output_topic)
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

        original_model_name = clean(payload.get("model_name", ""))

        if original_model_name:
            payload["resolver_used"] = False
            self.publish(payload)
            return

        if self.current_gt is None:
            rospy.logwarn("[PICK EVENT RESOLVER] No current GT. Cannot resolve model_name.")
            return

        age = time.time() - self.current_gt_wall_time

        if age > self.max_gt_age_sec:
            rospy.logwarn(
                "[PICK EVENT RESOLVER] Current GT is too old: %.2f sec. Event ignored.",
                age,
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

        rospy.loginfo(
            "[PICK EVENT RESOLVER] Resolved pick_event track_id=%s yolo_class=%s model=%s gt_class=%s",
            str(payload.get("track_id", "")),
            clean(payload.get("class_name", "")),
            resolved_model_name,
            gt_class_name,
        )

        self.publish(payload)

    def publish(self, payload: Dict):
        msg = String()
        msg.data = json.dumps(payload, sort_keys=True)
        self.pub.publish(msg)


def main():
    rospy.init_node("pick_event_model_resolver", anonymous=False)
    PickEventModelResolver()
    rospy.spin()


if __name__ == "__main__":
    main()

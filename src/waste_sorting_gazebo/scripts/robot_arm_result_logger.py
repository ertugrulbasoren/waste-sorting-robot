#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Robot Arm Result Logger.

Listens to:
  /waste/robot_arm_result

Writes:
  logs/trials/robot_arm_results.csv

Purpose:
  Log robot arm execution results for KPI analysis.

This logger is designed for the fake_robot_arm_executor.py stage, but it can
also be reused later with a real UR5/MoveIt executor if the output topic keeps
the same JSON structure.
"""

import csv
import json
import os
import time
from typing import Set

import rospy
from std_msgs.msg import String


VALID_CLASSES = ["glass", "metal", "paper", "plastic"]


def clean(value) -> str:
    return str(value or "").replace("\n", " ").replace("\r", " ").strip()


def project_root_from_script() -> str:
    return os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))


class RobotArmResultLogger:
    def __init__(self):
        project_root = project_root_from_script()

        self.result_topic = clean(rospy.get_param("~result_topic", "/waste/robot_arm_result"))
        self.log_path = clean(
            rospy.get_param(
                "~log_path",
                os.path.join(project_root, "logs", "trials", "robot_arm_results.csv"),
            )
        )

        self.ignore_duplicate_track_ids = bool(rospy.get_param("~ignore_duplicate_track_ids", True))
        self.ignore_duplicate_model_names = bool(rospy.get_param("~ignore_duplicate_model_names", True))

        self.logged_track_ids: Set[int] = set()
        self.logged_model_names: Set[str] = set()

        self.total_results = 0
        self.success_results = 0

        self.ensure_log_file()

        rospy.Subscriber(self.result_topic, String, self.result_callback, queue_size=50)

        rospy.loginfo("=" * 90)
        rospy.loginfo("[ROBOT ARM RESULT LOGGER] Started")
        rospy.loginfo("[ROBOT ARM RESULT LOGGER] result_topic: %s", self.result_topic)
        rospy.loginfo("[ROBOT ARM RESULT LOGGER] log_path: %s", self.log_path)
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
                    "timestamp",
                    "model_name",
                    "class_name",
                    "track_id",
                    "confidence",
                    "success",
                    "stage",
                    "message",
                    "cycle_time_sec",
                    "executor",
                    "event_type",
                ]
            )

    def result_callback(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except Exception:
            rospy.logwarn("[ROBOT ARM RESULT LOGGER] Invalid JSON result ignored.")
            return

        model_name = clean(payload.get("model_name", ""))
        class_name = clean(payload.get("class_name", ""))

        try:
            track_id = int(payload.get("track_id", -1))
        except Exception:
            track_id = -1

        if class_name and class_name not in VALID_CLASSES:
            rospy.logwarn("[ROBOT ARM RESULT LOGGER] Invalid class_name ignored: %s", class_name)
            return

        if self.ignore_duplicate_track_ids and track_id >= 0 and track_id in self.logged_track_ids:
            rospy.loginfo("[ROBOT ARM RESULT LOGGER] Duplicate track_id ignored: %s", str(track_id))
            return

        if self.ignore_duplicate_model_names and model_name and model_name in self.logged_model_names:
            rospy.loginfo("[ROBOT ARM RESULT LOGGER] Duplicate model_name ignored: %s", model_name)
            return

        success = bool(payload.get("success", False))
        stage = clean(payload.get("stage", ""))
        message = clean(payload.get("message", ""))
        executor = clean(payload.get("executor", ""))
        event_type = clean(payload.get("event_type", ""))

        try:
            confidence = float(payload.get("confidence", 0.0))
        except Exception:
            confidence = 0.0

        try:
            cycle_time_sec = float(payload.get("cycle_time_sec", 0.0))
        except Exception:
            cycle_time_sec = 0.0

        timestamp = payload.get("timestamp", "")

        self.append_row(
            timestamp=timestamp,
            model_name=model_name,
            class_name=class_name,
            track_id=track_id,
            confidence=confidence,
            success=success,
            stage=stage,
            message=message,
            cycle_time_sec=cycle_time_sec,
            executor=executor,
            event_type=event_type,
        )

        if track_id >= 0:
            self.logged_track_ids.add(track_id)

        if model_name:
            self.logged_model_names.add(model_name)

        self.total_results += 1
        if success:
            self.success_results += 1

        success_rate = (self.success_results / self.total_results * 100.0) if self.total_results else 0.0

        rospy.loginfo(
            "[ROBOT ARM RESULT LOGGER] Logged model=%s class=%s success=%s cycle=%.3f sec running_success=%.2f%% (%d/%d)",
            model_name,
            class_name,
            str(success),
            cycle_time_sec,
            success_rate,
            self.success_results,
            self.total_results,
        )

    def append_row(
        self,
        timestamp,
        model_name: str,
        class_name: str,
        track_id: int,
        confidence: float,
        success: bool,
        stage: str,
        message: str,
        cycle_time_sec: float,
        executor: str,
        event_type: str,
    ):
        with open(self.log_path, "a", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(
                [
                    round(time.time(), 3),
                    timestamp,
                    model_name,
                    class_name,
                    track_id,
                    round(confidence, 6),
                    success,
                    stage,
                    message,
                    round(cycle_time_sec, 6),
                    executor,
                    event_type,
                ]
            )


def main():
    rospy.init_node("robot_arm_result_logger", anonymous=False)
    RobotArmResultLogger()
    rospy.spin()


if __name__ == "__main__":
    main()

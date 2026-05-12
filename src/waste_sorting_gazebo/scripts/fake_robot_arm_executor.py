#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Fake Robot Arm Executor.

This node is an intermediate robot-arm integration layer.

It listens to:
  /waste/pick_event_resolved

It simulates a robot arm pick-and-place cycle:
  1. Receive resolved pick event
  2. Move to pick pose
  3. Grasp object
  4. Move to target bin
  5. Place object
  6. Publish robot action result

For now, this node still uses Gazebo SetModelState to move the object.
The purpose is to separate "robot arm execution logic" from the old direct
sort executor logic.

Later this file can be replaced by a real UR5/MoveIt executor while keeping
the same input topic:
  /waste/pick_event_resolved
"""

import json
import math
import time
from typing import Dict, Tuple

import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState, SetModelState
from geometry_msgs.msg import Point, Pose, Quaternion, Twist
from std_msgs.msg import String


VALID_CLASSES = ["glass", "metal", "paper", "plastic"]


def clean(value) -> str:
    return str(value or "").replace("\n", " ").replace("\r", " ").strip()


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class FakeRobotArmExecutor:
    def __init__(self):
        self.pick_event_topic = clean(rospy.get_param("~pick_event_topic", "/waste/pick_event_resolved"))
        self.result_topic = clean(rospy.get_param("~result_topic", "/waste/robot_arm_result"))

        self.arm_model_name = clean(rospy.get_param("~arm_model_name", "fake_sorting_arm"))

        self.pick_delay_sec = float(rospy.get_param("~pick_delay_sec", 0.35))
        self.grasp_delay_sec = float(rospy.get_param("~grasp_delay_sec", 0.25))
        self.move_delay_sec = float(rospy.get_param("~move_delay_sec", 0.45))
        self.place_delay_sec = float(rospy.get_param("~place_delay_sec", 0.25))

        self.lift_z = float(rospy.get_param("~lift_z", 0.55))
        self.drop_z = float(rospy.get_param("~drop_z", 0.20))

        self.duplicate_guard_sec = float(rospy.get_param("~duplicate_guard_sec", 5.0))

        self.bin_positions: Dict[str, Tuple[float, float, float]] = {
            "metal": (
                float(rospy.get_param("~metal_bin_x", 1.5)),
                float(rospy.get_param("~metal_bin_y", -0.90)),
                self.drop_z,
            ),
            "paper": (
                float(rospy.get_param("~paper_bin_x", 1.5)),
                float(rospy.get_param("~paper_bin_y", -0.45)),
                self.drop_z,
            ),
            "glass": (
                float(rospy.get_param("~glass_bin_x", 1.5)),
                float(rospy.get_param("~glass_bin_y", 0.00)),
                self.drop_z,
            ),
            "plastic": (
                float(rospy.get_param("~plastic_bin_x", 1.5)),
                float(rospy.get_param("~plastic_bin_y", 0.45)),
                self.drop_z,
            ),
        }

        self.last_handled_by_model: Dict[str, float] = {}
        self.last_handled_by_track: Dict[int, float] = {}

        self.busy = False

        self.result_pub = rospy.Publisher(self.result_topic, String, queue_size=20)

        rospy.loginfo("[FAKE ARM EXECUTOR] Waiting for Gazebo services...")
        rospy.wait_for_service("/gazebo/get_model_state")
        rospy.wait_for_service("/gazebo/set_model_state")

        self.get_model_state_srv = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
        self.set_model_state_srv = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)

        rospy.Subscriber(self.pick_event_topic, String, self.pick_event_callback, queue_size=20)

        rospy.loginfo("=" * 90)
        rospy.loginfo("[FAKE ARM EXECUTOR] Started")
        rospy.loginfo("[FAKE ARM EXECUTOR] pick_event_topic: %s", self.pick_event_topic)
        rospy.loginfo("[FAKE ARM EXECUTOR] result_topic: %s", self.result_topic)
        rospy.loginfo("[FAKE ARM EXECUTOR] arm_model_name: %s", self.arm_model_name)
        rospy.loginfo("[FAKE ARM EXECUTOR] bin_positions: %s", str(self.bin_positions))
        rospy.loginfo("=" * 90)

    def pick_event_callback(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except Exception:
            rospy.logwarn("[FAKE ARM EXECUTOR] Invalid JSON pick event ignored.")
            return

        if self.busy:
            rospy.logwarn("[FAKE ARM EXECUTOR] Arm is busy. Event ignored.")
            return

        model_name = clean(payload.get("model_name", ""))
        class_name = clean(payload.get("class_name", ""))

        try:
            track_id = int(payload.get("track_id", -1))
        except Exception:
            track_id = -1

        if not model_name:
            rospy.logwarn("[FAKE ARM EXECUTOR] Empty model_name. Event ignored.")
            return

        if class_name not in VALID_CLASSES:
            rospy.logwarn("[FAKE ARM EXECUTOR] Invalid class_name=%s. Event ignored.", class_name)
            return

        now = time.time()

        if model_name in self.last_handled_by_model:
            age = now - self.last_handled_by_model[model_name]
            if age < self.duplicate_guard_sec:
                rospy.loginfo("[FAKE ARM EXECUTOR] Duplicate model ignored: %s", model_name)
                return

        if track_id >= 0 and track_id in self.last_handled_by_track:
            age = now - self.last_handled_by_track[track_id]
            if age < self.duplicate_guard_sec:
                rospy.loginfo("[FAKE ARM EXECUTOR] Duplicate track ignored: %s", str(track_id))
                return

        self.busy = True

        try:
            self.execute_cycle(payload, model_name, class_name, track_id)
            self.last_handled_by_model[model_name] = time.time()
            if track_id >= 0:
                self.last_handled_by_track[track_id] = time.time()
        finally:
            self.busy = False

    def execute_cycle(self, payload: Dict, model_name: str, class_name: str, track_id: int):
        cycle_start = time.time()

        rospy.loginfo("=" * 90)
        rospy.loginfo(
            "[FAKE ARM EXECUTOR] PICK EVENT received track_id=%s class=%s model=%s confidence=%.3f",
            str(track_id),
            class_name,
            model_name,
            float(payload.get("confidence", 0.0)),
        )

        current_pose = self.get_model_pose(model_name)

        if current_pose is None:
            rospy.logwarn("[FAKE ARM EXECUTOR] Model state not found: %s", model_name)
            self.publish_result(
                payload=payload,
                success=False,
                stage="get_model_state_failed",
                message="Model state not found",
                cycle_time_sec=time.time() - cycle_start,
            )
            return

        pick_x = current_pose.position.x
        pick_y = current_pose.position.y
        pick_z = current_pose.position.z

        rospy.loginfo(
            "[FAKE ARM EXECUTOR] ARM MOVING TO PICK pose=(%.3f, %.3f, %.3f)",
            pick_x,
            pick_y,
            pick_z,
        )
        rospy.sleep(self.pick_delay_sec)

        rospy.loginfo("[FAKE ARM EXECUTOR] ARM GRASPING object=%s", model_name)
        rospy.sleep(self.grasp_delay_sec)

        rospy.loginfo("[FAKE ARM EXECUTOR] ARM LIFTING object=%s to z=%.3f", model_name, self.lift_z)
        self.set_model_pose(model_name, pick_x, pick_y, self.lift_z)
        rospy.sleep(self.move_delay_sec)

        target_x, target_y, target_z = self.bin_positions[class_name]

        rospy.loginfo(
            "[FAKE ARM EXECUTOR] ARM MOVING TO BIN class=%s target=(%.3f, %.3f, %.3f)",
            class_name,
            target_x,
            target_y,
            target_z,
        )
        self.set_model_pose(model_name, target_x, target_y, self.lift_z)
        rospy.sleep(self.move_delay_sec)

        rospy.loginfo(
            "[FAKE ARM EXECUTOR] ARM PLACING object=%s at bin=(%.3f, %.3f, %.3f)",
            model_name,
            target_x,
            target_y,
            target_z,
        )
        self.set_model_pose(model_name, target_x, target_y, target_z)
        rospy.sleep(self.place_delay_sec)

        cycle_time = time.time() - cycle_start

        rospy.loginfo(
            "[FAKE ARM EXECUTOR] SORT COMPLETE model=%s class=%s cycle_time=%.3f sec",
            model_name,
            class_name,
            cycle_time,
        )
        rospy.loginfo("=" * 90)

        self.publish_result(
            payload=payload,
            success=True,
            stage="complete",
            message="Fake robot arm pick-and-place completed",
            cycle_time_sec=cycle_time,
        )

    def get_model_pose(self, model_name: str):
        try:
            response = self.get_model_state_srv(model_name, "world")
            if not response.success:
                return None
            return response.pose
        except Exception as exc:
            rospy.logwarn("[FAKE ARM EXECUTOR] get_model_state failed: %s", str(exc))
            return None

    def set_model_pose(self, model_name: str, x: float, y: float, z: float) -> bool:
        state = ModelState()
        state.model_name = model_name
        state.reference_frame = "world"
        state.pose = Pose()
        state.pose.position = Point(x, y, z)
        state.pose.orientation = yaw_to_quaternion(0.0)
        state.twist = Twist()

        try:
            response = self.set_model_state_srv(state)
            if not response.success:
                rospy.logwarn("[FAKE ARM EXECUTOR] set_model_state failed: %s", response.status_message)
            return bool(response.success)
        except Exception as exc:
            rospy.logwarn("[FAKE ARM EXECUTOR] set_model_state exception: %s", str(exc))
            return False

    def publish_result(
        self,
        payload: Dict,
        success: bool,
        stage: str,
        message: str,
        cycle_time_sec: float,
    ):
        result = {
            "event_type": "robot_arm_result",
            "success": success,
            "stage": stage,
            "message": message,
            "cycle_time_sec": round(cycle_time_sec, 3),
            "model_name": clean(payload.get("model_name", "")),
            "class_name": clean(payload.get("class_name", "")),
            "track_id": payload.get("track_id", -1),
            "confidence": payload.get("confidence", 0.0),
            "timestamp": time.time(),
            "executor": "fake_robot_arm_executor",
        }

        msg = String()
        msg.data = json.dumps(result, sort_keys=True)
        self.result_pub.publish(msg)


def main():
    rospy.init_node("fake_robot_arm_executor", anonymous=False)
    FakeRobotArmExecutor()
    rospy.spin()


if __name__ == "__main__":
    main()

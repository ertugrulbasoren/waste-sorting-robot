#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import math
import sys
import time
import traceback
from typing import Dict, List, Optional, Set, Tuple

import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState, GetWorldProperties, SetModelState
from geometry_msgs.msg import Pose, Quaternion, Twist, Vector3
from std_msgs.msg import String


VALID_CLASSES = {"glass", "metal", "paper", "plastic"}


class SortExecutorNode:
    def __init__(self) -> None:
        rospy.init_node("sort_executor", anonymous=False)

        self.pick_event_topic = rospy.get_param("~pick_event_topic", "/waste/pick_event")

        self.drop_z = float(rospy.get_param("~drop_z", 0.35))
        self.wait_for_model_timeout_sec = float(rospy.get_param("~wait_for_model_timeout_sec", 1.2))

        self.bin_positions: Dict[str, Tuple[float, float, float]] = {
            "glass": (
                float(rospy.get_param("~glass_bin_x", 1.50)),
                float(rospy.get_param("~glass_bin_y", -0.60)),
                self.drop_z,
            ),
            "metal": (
                float(rospy.get_param("~metal_bin_x", 1.50)),
                float(rospy.get_param("~metal_bin_y", -0.20)),
                self.drop_z,
            ),
            "paper": (
                float(rospy.get_param("~paper_bin_x", 1.50)),
                float(rospy.get_param("~paper_bin_y", 0.20)),
                self.drop_z,
            ),
            "plastic": (
                float(rospy.get_param("~plastic_bin_x", 1.50)),
                float(rospy.get_param("~plastic_bin_y", 0.60)),
                self.drop_z,
            ),
        }

        self.model_prefixes: Dict[str, List[str]] = {
            "glass": ["waste_glass", "trash_glass", "glass"],
            "metal": ["waste_metal", "trash_metal", "metal", "can"],
            "paper": ["waste_paper", "trash_paper", "paper", "cardboard"],
            "plastic": ["waste_plastic", "trash_plastic", "plastic", "bottle"],
        }

        self.processed_track_ids: Set[int] = set()
        self.processed_model_names: Set[str] = set()

        rospy.loginfo("[SORT EXECUTOR] Waiting for Gazebo services...")
        rospy.wait_for_service("/gazebo/set_model_state")
        rospy.wait_for_service("/gazebo/get_world_properties")
        rospy.wait_for_service("/gazebo/get_model_state")

        self.set_model_state_srv = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        self.get_world_properties_srv = rospy.ServiceProxy("/gazebo/get_world_properties", GetWorldProperties)
        self.get_model_state_srv = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)

        self.sub_pick_event = rospy.Subscriber(
            self.pick_event_topic,
            String,
            self.pick_event_callback,
            queue_size=20,
        )

        rospy.loginfo("=" * 80)
        rospy.loginfo("[SORT EXECUTOR] Started")
        rospy.loginfo("[SORT EXECUTOR] pick_event_topic: %s", self.pick_event_topic)
        rospy.loginfo("[SORT EXECUTOR] wait_for_model_timeout_sec: %.2f", self.wait_for_model_timeout_sec)
        rospy.loginfo("[SORT EXECUTOR] bin_positions: %s", self.bin_positions)
        rospy.loginfo("=" * 80)

    def pick_event_callback(self, msg: String) -> None:
        try:
            event = json.loads(msg.data)
        except Exception as exc:
            rospy.logwarn("[SORT EXECUTOR] Invalid JSON pick_event: %s", str(exc))
            return

        if not isinstance(event, dict):
            return

        class_name = str(event.get("class_name", "")).strip().lower()
        if class_name not in VALID_CLASSES:
            rospy.logwarn("[SORT EXECUTOR] Unknown class_name ignored: %s", class_name)
            return

        track_id = self._safe_int(event.get("track_id", -1), -1)
        if track_id < 0:
            rospy.logwarn("[SORT EXECUTOR] Invalid track_id ignored: %s", event)
            return

        model_name_from_event = str(event.get("model_name", "")).strip()

        if track_id in self.processed_track_ids:
            rospy.logwarn("[SORT EXECUTOR] Duplicate track_id ignored: %d", track_id)
            return

        if model_name_from_event and model_name_from_event in self.processed_model_names:
            rospy.logwarn("[SORT EXECUTOR] Duplicate model_name ignored: %s", model_name_from_event)
            return

        rospy.loginfo(
            "[SORT EXECUTOR] Received pick_event track_id=%d class=%s model=%s center=(%s,%s)",
            track_id,
            class_name,
            model_name_from_event if model_name_from_event else "?",
            str(event.get("center_x", "?")),
            str(event.get("center_y", "?")),
        )

        model_name = self.resolve_model_name(class_name, model_name_from_event)

        if model_name is None:
            rospy.logwarn(
                "[SORT EXECUTOR] Could not find Gazebo model for class=%s event_model=%s. Event ignored.",
                class_name,
                model_name_from_event,
            )
            return

        success = self.move_model_to_bin(model_name, class_name)

        if success:
            self.processed_track_ids.add(track_id)
            self.processed_model_names.add(model_name)

            rospy.loginfo(
                "[SORT EXECUTOR] Sorted model=%s class=%s track_id=%d",
                model_name,
                class_name,
                track_id,
            )
        else:
            rospy.logwarn(
                "[SORT EXECUTOR] Failed to sort model=%s class=%s track_id=%d",
                model_name,
                class_name,
                track_id,
            )

    def resolve_model_name(self, class_name: str, event_model_name: str = "") -> Optional[str]:
        if event_model_name:
            if self.wait_until_model_exists(event_model_name, self.wait_for_model_timeout_sec):
                return event_model_name

            rospy.logwarn(
                "[SORT EXECUTOR] Event model_name=%s not found after wait. Falling back to class search.",
                event_model_name,
            )

        model_names = self.get_world_model_names()
        if not model_names:
            return None

        candidate_names = []
        prefixes = self.model_prefixes.get(class_name, [class_name])

        for model_name in model_names:
            lower = model_name.lower()

            if self._is_non_waste_model(lower):
                continue

            for prefix in prefixes:
                if prefix.lower() in lower:
                    candidate_names.append(model_name)
                    break

        if not candidate_names:
            rospy.logwarn(
                "[SORT EXECUTOR] No direct model candidate for class=%s. World models: %s",
                class_name,
                model_names,
            )
            return None

        best_name = None
        best_score = float("inf")

        for candidate in candidate_names:
            pose = self.get_model_pose(candidate)
            if pose is None:
                continue

            score = self._conveyor_pick_score(pose)
            if score < best_score:
                best_score = score
                best_name = candidate

        if best_name is None:
            best_name = candidate_names[0]

        rospy.loginfo(
            "[SORT EXECUTOR] Resolved class=%s to model=%s candidates=%s",
            class_name,
            best_name,
            candidate_names,
        )

        return best_name

    def wait_until_model_exists(self, model_name: str, timeout_sec: float) -> bool:
        start = time.time()

        while time.time() - start <= timeout_sec and not rospy.is_shutdown():
            model_names = self.get_world_model_names()

            if model_name in model_names:
                return True

            rospy.sleep(0.05)

        return False

    def move_model_to_bin(self, model_name: str, class_name: str) -> bool:
        if class_name not in self.bin_positions:
            return False

        x, y, z = self.bin_positions[class_name]

        state = ModelState()
        state.model_name = model_name
        state.reference_frame = "world"

        state.pose = Pose()
        state.pose.position.x = float(x)
        state.pose.position.y = float(y)
        state.pose.position.z = float(z)
        state.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)

        state.twist = Twist()
        state.twist.linear = Vector3(0.0, 0.0, 0.0)
        state.twist.angular = Vector3(0.0, 0.0, 0.0)

        try:
            response = self.set_model_state_srv(state)
        except Exception as exc:
            rospy.logerr("[SORT EXECUTOR] /gazebo/set_model_state failed: %s", str(exc))
            return False

        if not response.success:
            rospy.logwarn("[SORT EXECUTOR] Gazebo rejected set_model_state: %s", response.status_message)
            return False

        return True

    def get_world_model_names(self) -> List[str]:
        try:
            response = self.get_world_properties_srv()
        except Exception:
            return []

        return list(response.model_names)

    def get_model_pose(self, model_name: str) -> Optional[Pose]:
        try:
            response = self.get_model_state_srv(model_name, "world")
        except Exception:
            return None

        if not response.success:
            return None

        return response.pose

    @staticmethod
    def _is_non_waste_model(lower_model_name: str) -> bool:
        blocked_keywords = [
            "ground",
            "plane",
            "camera",
            "bin",
            "trash_bin",
            "conveyor",
            "belt",
            "robot",
            "arm",
            "ur5",
            "world",
            "light",
            "sun",
        ]

        for keyword in blocked_keywords:
            if keyword in lower_model_name:
                return True

        return False

    @staticmethod
    def _conveyor_pick_score(pose: Pose) -> float:
        x = float(pose.position.x)
        y = float(pose.position.y)
        z = float(pose.position.z)
        return math.sqrt((x - 0.4) ** 2 + (y - 0.0) ** 2) + abs(z - 0.1)

    @staticmethod
    def _safe_int(value, default: int) -> int:
        try:
            return int(value)
        except Exception:
            return default


def main() -> None:
    try:
        SortExecutorNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as exc:
        rospy.logfatal("[SORT EXECUTOR] Fatal error: %s", str(exc))
        rospy.logfatal(traceback.format_exc())
        sys.exit(1)


if __name__ == "__main__":
    main()

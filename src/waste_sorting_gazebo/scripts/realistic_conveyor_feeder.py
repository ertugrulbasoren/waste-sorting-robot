#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Kinematic realistic 3D waste feeder for conveyor-based YOLO testing.

This node spawns realistic Objaverse-derived 3D waste assets and moves them
along the conveyor using /gazebo/set_model_state.

Why kinematic:
  Some imported realistic OBJ assets do not behave consistently with Gazebo
  physics/collision. Instead of changing the world file, this node keeps the
  asset stable and visually moves it like it is flowing on a conveyor.

Important:
  This node does NOT publish /waste/pick_event.
  YOLO + tracker must detect the object and produce the pick_event.

Main visual-height fix:
  Different OBJ assets have different local mesh origins.
  A fixed spawn_z causes some assets to float and some to sink.
  This node reads meshes/model.obj, computes the minimum local vertex z,
  and places the visual bottom at object_bottom_z.

Pipeline:
  realistic_conveyor_feeder.py
      ↓
  /top_camera/image_raw
      ↓
  yolo_detector.py
      ↓
  /waste/detections
      ↓
  tracker_node.py
      ↓
  /waste/pick_event
      ↓
  sort_executor.py
"""

import json
import math
import os
import random
import re
import sys
import time
import traceback
from typing import Dict, List, Optional, Tuple

import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import DeleteModel, GetWorldProperties, SetModelState, SpawnModel
from geometry_msgs.msg import Point, Pose, Quaternion, Twist
from std_msgs.msg import String


VALID_CLASSES = ["glass", "metal", "paper", "plastic"]


def package_root_from_script() -> str:
    return os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))


def clean(value) -> str:
    return str(value or "").replace("\n", " ").replace("\r", " ").strip()


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def load_json(path: str) -> Dict:
    if not os.path.exists(path):
        return {}

    try:
        with open(path, "r") as f:
            return json.load(f)
    except Exception:
        return {}


def list_prepared_assets(realistic_root: str) -> Dict[str, List[str]]:
    assets = {class_name: [] for class_name in VALID_CLASSES}

    for class_name in VALID_CLASSES:
        class_dir = os.path.join(realistic_root, class_name)

        if not os.path.isdir(class_dir):
            continue

        for folder in sorted(os.listdir(class_dir)):
            asset_dir = os.path.join(class_dir, folder)

            if not os.path.isdir(asset_dir):
                continue

            if folder.startswith("."):
                continue

            sdf_path = os.path.join(asset_dir, "model.sdf")
            obj_path = os.path.join(asset_dir, "meshes", "model.obj")

            if os.path.exists(sdf_path) and os.path.exists(obj_path):
                assets[class_name].append(folder)

    return assets


def load_sdf(asset_dir: str) -> str:
    sdf_path = os.path.join(asset_dir, "model.sdf")

    if not os.path.exists(sdf_path):
        raise FileNotFoundError("model.sdf not found: {}".format(sdf_path))

    with open(sdf_path, "r") as f:
        return f.read()


def patch_sdf_for_spawn(sdf_xml: str, asset_dir: str, static_model: bool) -> str:
    mesh_path = os.path.abspath(os.path.join(asset_dir, "meshes", "model.obj"))

    if not os.path.exists(mesh_path):
        raise FileNotFoundError("model.obj not found: {}".format(mesh_path))

    mesh_uri = "file://{}".format(mesh_path)

    sdf_xml = re.sub(
        r"<uri>.*?</uri>",
        "<uri>{}</uri>".format(mesh_uri),
        sdf_xml,
        count=1,
        flags=re.DOTALL,
    )

    static_text = "true" if static_model else "false"

    if re.search(r"<static>.*?</static>", sdf_xml, flags=re.DOTALL):
        sdf_xml = re.sub(
            r"<static>.*?</static>",
            "<static>{}</static>".format(static_text),
            sdf_xml,
            count=1,
            flags=re.DOTALL,
        )

    return sdf_xml


def delete_model_safe(delete_srv, model_name: str) -> None:
    try:
        delete_srv(model_name)
    except Exception:
        pass


def delete_all_waste_models(world_srv, delete_srv) -> None:
    try:
        response = world_srv()
        names = list(response.model_names)
    except Exception:
        return

    for name in names:
        if name.startswith("waste_"):
            delete_model_safe(delete_srv, name)


def read_obj_z_bounds(obj_path: str) -> Tuple[float, float]:
    """
    Reads local z bounds from an OBJ file.

    OBJ vertex lines look like:
      v x y z

    Returns:
      min_z, max_z
    """

    min_z = None
    max_z = None

    with open(obj_path, "r", errors="ignore") as f:
        for line in f:
            if not line.startswith("v "):
                continue

            parts = line.strip().split()

            if len(parts) < 4:
                continue

            try:
                z = float(parts[3])
            except Exception:
                continue

            if min_z is None or z < min_z:
                min_z = z

            if max_z is None or z > max_z:
                max_z = z

    if min_z is None or max_z is None:
        return 0.0, 0.20

    return float(min_z), float(max_z)


class RealisticConveyorFeeder:
    def __init__(self):
        self.package_root = package_root_from_script()
        self.realistic_root = os.path.join(
            self.package_root,
            "models",
            "realistic_assets",
        )

        self.spawn_interval_sec = float(rospy.get_param("~spawn_interval_sec", 7.0))

        self.spawn_x = float(rospy.get_param("~spawn_x", -0.85))
        self.spawn_y = float(rospy.get_param("~spawn_y", 0.0))

        # This is the real visual target:
        # local OBJ bottom will be placed at this world z.
        self.object_bottom_z = float(rospy.get_param("~object_bottom_z", 0.145))

        # Optional extra visual lift. Use this for quick tuning.
        self.visual_lift = float(rospy.get_param("~visual_lift", 0.0))

        self.end_x = float(rospy.get_param("~end_x", 0.45))
        self.conveyor_speed = float(rospy.get_param("~conveyor_speed", 0.22))
        self.update_rate = float(rospy.get_param("~update_rate", 30.0))

        self.belt_y_min = float(rospy.get_param("~belt_y_min", -0.035))
        self.belt_y_max = float(rospy.get_param("~belt_y_max", 0.035))

        self.random_y = bool(rospy.get_param("~random_y", True))
        self.random_yaw = bool(rospy.get_param("~random_yaw", True))

        # Intentional: keep static and move kinematically.
        self.static_model = bool(rospy.get_param("~static_model", True))

        self.delete_before_spawn = bool(rospy.get_param("~delete_before_spawn", True))

        self.class_mode = clean(rospy.get_param("~class_mode", "balanced")).lower()

        self.publish_ground_truth = bool(rospy.get_param("~publish_ground_truth", True))
        self.ground_truth_topic = clean(rospy.get_param("~ground_truth_topic", "/waste/current_ground_truth"))

        self.pick_event_topic = clean(rospy.get_param("~pick_event_topic", "/waste/pick_event"))

        self.assets = list_prepared_assets(self.realistic_root)

        self.gt_pub = rospy.Publisher(self.ground_truth_topic, String, queue_size=10, latch=True)

        self.active_model_name: Optional[str] = None
        self.active_class_name: Optional[str] = None
        self.active_x = self.spawn_x
        self.active_y = self.spawn_y
        self.active_z = self.object_bottom_z
        self.active_yaw = 0.0
        self.active_sorted = False

        self.sequence_id = 0
        self.last_spawn_wall_time = 0.0

        rospy.Subscriber(self.pick_event_topic, String, self.pick_event_callback, queue_size=10)

        rospy.loginfo("=" * 90)
        rospy.loginfo("[REALISTIC FEEDER] Started - KINEMATIC MODE WITH OBJ Z AUTO-ALIGN")
        rospy.loginfo("[REALISTIC FEEDER] realistic_root: %s", self.realistic_root)
        rospy.loginfo("[REALISTIC FEEDER] spawn_interval_sec: %.2f", self.spawn_interval_sec)
        rospy.loginfo("[REALISTIC FEEDER] spawn_x: %.3f", self.spawn_x)
        rospy.loginfo("[REALISTIC FEEDER] object_bottom_z: %.3f", self.object_bottom_z)
        rospy.loginfo("[REALISTIC FEEDER] visual_lift: %.3f", self.visual_lift)
        rospy.loginfo("[REALISTIC FEEDER] end_x: %.3f", self.end_x)
        rospy.loginfo("[REALISTIC FEEDER] conveyor_speed: %.3f", self.conveyor_speed)
        rospy.loginfo("[REALISTIC FEEDER] static_model: %s", self.static_model)
        rospy.loginfo("[REALISTIC FEEDER] class_mode: %s", self.class_mode)

        for class_name in VALID_CLASSES:
            rospy.loginfo("[REALISTIC FEEDER] %s assets: %d", class_name, len(self.assets[class_name]))

            if not self.assets[class_name]:
                rospy.logerr("[REALISTIC FEEDER] No prepared assets for class=%s", class_name)
                sys.exit(1)

        rospy.loginfo("=" * 90)

        rospy.loginfo("[REALISTIC FEEDER] Waiting for Gazebo services...")
        rospy.wait_for_service("/gazebo/spawn_sdf_model")
        rospy.wait_for_service("/gazebo/delete_model")
        rospy.wait_for_service("/gazebo/get_world_properties")
        rospy.wait_for_service("/gazebo/set_model_state")

        self.spawn_srv = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
        self.delete_srv = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
        self.world_srv = rospy.ServiceProxy("/gazebo/get_world_properties", GetWorldProperties)
        self.set_state_srv = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)

    def choose_class(self) -> str:
        if self.class_mode == "balanced":
            return VALID_CLASSES[self.sequence_id % len(VALID_CLASSES)]

        return random.choice(VALID_CLASSES)

    def publish_gt(
        self,
        model_name: str,
        class_name: str,
        asset_folder: str,
        asset_name: str,
        uid: str,
        obj_min_z: float,
        obj_max_z: float,
        world_z: float,
    ):
        if not self.publish_ground_truth:
            return

        payload = {
            "model_name": model_name,
            "class_name": class_name,
            "asset_folder": asset_folder,
            "asset_name": asset_name,
            "uid": uid,
            "sequence_id": self.sequence_id,
            "stamp": time.time(),
            "motion_mode": "kinematic_set_model_state",
            "object_bottom_z": self.object_bottom_z,
            "visual_lift": self.visual_lift,
            "obj_min_z": obj_min_z,
            "obj_max_z": obj_max_z,
            "world_z": world_z,
        }

        msg = String()
        msg.data = json.dumps(payload)
        self.gt_pub.publish(msg)

    def pick_event_callback(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except Exception:
            return

        model_name = clean(payload.get("model_name", ""))

        if self.active_model_name and model_name == self.active_model_name:
            self.active_sorted = True
            rospy.loginfo("[REALISTIC FEEDER] Active model sorted, stopping kinematic control: %s", model_name)

    def set_model_pose(self, model_name: str, x: float, y: float, z: float, yaw: float) -> bool:
        state = ModelState()
        state.model_name = model_name
        state.reference_frame = "world"

        state.pose.position = Point(x, y, z)
        state.pose.orientation = yaw_to_quaternion(yaw)

        state.twist = Twist()

        try:
            response = self.set_state_srv(state)
            return bool(response.success)
        except Exception as exc:
            rospy.logwarn("[REALISTIC FEEDER] SetModelState failed for %s: %s", model_name, str(exc))
            return False

    def compute_world_z_for_asset(self, asset_dir: str) -> Tuple[float, float, float]:
        obj_path = os.path.join(asset_dir, "meshes", "model.obj")
        obj_min_z, obj_max_z = read_obj_z_bounds(obj_path)

        # Put OBJ visual bottom exactly at object_bottom_z.
        world_z = self.object_bottom_z + self.visual_lift - obj_min_z

        return world_z, obj_min_z, obj_max_z

    def spawn_once(self):
        if self.delete_before_spawn:
            delete_all_waste_models(self.world_srv, self.delete_srv)
            rospy.sleep(0.20)

        class_name = self.choose_class()
        asset_folder = random.choice(self.assets[class_name])
        asset_dir = os.path.join(self.realistic_root, class_name, asset_folder)

        metadata = load_json(os.path.join(asset_dir, "metadata.json"))
        asset_name = clean(metadata.get("asset_name", asset_folder))
        uid = clean(metadata.get("uid", ""))

        model_name = "waste_{}_{:03d}".format(class_name, self.sequence_id + 1)

        y = random.uniform(self.belt_y_min, self.belt_y_max) if self.random_y else self.spawn_y
        yaw = random.uniform(-math.pi, math.pi) if self.random_yaw else 0.0

        world_z, obj_min_z, obj_max_z = self.compute_world_z_for_asset(asset_dir)

        sdf_xml = load_sdf(asset_dir)
        sdf_xml = patch_sdf_for_spawn(
            sdf_xml=sdf_xml,
            asset_dir=asset_dir,
            static_model=self.static_model,
        )

        pose = Pose()
        pose.position = Point(self.spawn_x, y, world_z)
        pose.orientation = yaw_to_quaternion(yaw)

        response = self.spawn_srv(
            model_name,
            sdf_xml,
            "",
            pose,
            "world",
        )

        if not response.success:
            rospy.logerr("[REALISTIC FEEDER] Spawn failed model=%s status=%s", model_name, response.status_message)
            return

        self.active_model_name = model_name
        self.active_class_name = class_name
        self.active_x = self.spawn_x
        self.active_y = y
        self.active_z = world_z
        self.active_yaw = yaw
        self.active_sorted = False

        self.publish_gt(
            model_name=model_name,
            class_name=class_name,
            asset_folder=asset_folder,
            asset_name=asset_name,
            uid=uid,
            obj_min_z=obj_min_z,
            obj_max_z=obj_max_z,
            world_z=world_z,
        )

        rospy.loginfo(
            "[REALISTIC FEEDER] Spawned seq=%d model=%s class=%s asset=%s asset_name=%s "
            "pose=(%.3f, %.3f, %.3f) yaw=%.3f obj_z=[%.3f, %.3f] static=%s",
            self.sequence_id + 1,
            model_name,
            class_name,
            asset_folder,
            asset_name,
            self.spawn_x,
            y,
            world_z,
            yaw,
            obj_min_z,
            obj_max_z,
            self.static_model,
        )

        self.sequence_id += 1
        self.last_spawn_wall_time = time.time()

    def update_active_motion(self, dt: float):
        if not self.active_model_name:
            return

        if self.active_sorted:
            return

        self.active_x += self.conveyor_speed * dt

        if self.active_x > self.end_x:
            rospy.loginfo("[REALISTIC FEEDER] Active model reached end_x: %s", self.active_model_name)
            self.active_sorted = True
            return

        self.set_model_pose(
            model_name=self.active_model_name,
            x=self.active_x,
            y=self.active_y,
            z=self.active_z,
            yaw=self.active_yaw,
        )

    def run(self):
        rate = rospy.Rate(self.update_rate)

        rospy.sleep(1.0)

        last_time = time.time()
        self.spawn_once()

        while not rospy.is_shutdown():
            now = time.time()
            dt = max(0.0, min(now - last_time, 0.20))
            last_time = now

            try:
                self.update_active_motion(dt)

                if now - self.last_spawn_wall_time >= self.spawn_interval_sec:
                    self.spawn_once()

            except Exception as exc:
                rospy.logerr("[REALISTIC FEEDER] loop error: %s", str(exc))
                rospy.logerr(traceback.format_exc())

            rate.sleep()


def main():
    rospy.init_node("realistic_conveyor_feeder", anonymous=False)
    node = RealisticConveyorFeeder()
    node.run()


if __name__ == "__main__":
    main()

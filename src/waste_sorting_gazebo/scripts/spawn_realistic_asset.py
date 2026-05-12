#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Spawn prepared realistic Objaverse waste assets into Gazebo.

This version expects OBJ-prepared assets:

  models/realistic_assets/<class>/<asset_folder>/
    model.sdf
    model.config
    meshes/model.obj

Spawned model names remain compatible with sort_executor.py:

  waste_glass_001
  waste_metal_001
  waste_paper_001
  waste_plastic_001
"""

import json
import math
import os
import random
import re
import sys
import traceback
from typing import Dict, List, Optional

import rospy
from gazebo_msgs.srv import DeleteModel, GetWorldProperties, SpawnModel
from geometry_msgs.msg import Point, Pose, Quaternion


VALID_CLASSES = ["glass", "metal", "paper", "plastic"]

DEFAULT_X = -0.55
DEFAULT_Y = 0.0
DEFAULT_Z = 0.11

DEFAULT_BELT_Y_MIN = -0.045
DEFAULT_BELT_Y_MAX = 0.045


def package_root_from_script() -> str:
    return os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))


def clean(value) -> str:
    return str(value or "").replace("\n", " ").replace("\r", " ").strip()


def clamp(value: float, min_value: float, max_value: float) -> float:
    return max(min_value, min(max_value, value))


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def list_asset_folders(class_dir: str) -> List[str]:
    if not os.path.isdir(class_dir):
        return []

    result = []

    for name in sorted(os.listdir(class_dir)):
        path = os.path.join(class_dir, name)

        if not os.path.isdir(path):
            continue

        if name.startswith("."):
            continue

        model_sdf = os.path.join(path, "model.sdf")
        model_config = os.path.join(path, "model.config")
        mesh_obj = os.path.join(path, "meshes", "model.obj")

        if os.path.exists(model_sdf) and os.path.exists(model_config) and os.path.exists(mesh_obj):
            result.append(name)

    return result


def load_metadata(asset_dir: str) -> Dict:
    path = os.path.join(asset_dir, "metadata.json")

    if not os.path.exists(path):
        return {}

    try:
        with open(path, "r") as f:
            return json.load(f)
    except Exception:
        return {}


def load_sdf(asset_dir: str) -> str:
    path = os.path.join(asset_dir, "model.sdf")

    if not os.path.exists(path):
        raise FileNotFoundError("model.sdf not found: {}".format(path))

    with open(path, "r") as f:
        return f.read()


def patch_sdf(
    sdf_xml: str,
    asset_dir: str,
    static_model: bool,
) -> str:
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


def delete_model_if_exists(delete_srv, model_name: str) -> None:
    try:
        delete_srv(model_name)
    except Exception:
        pass


def delete_existing_waste_models(world_props_srv, delete_srv, class_name: Optional[str] = None) -> None:
    try:
        response = world_props_srv()
        model_names = list(response.model_names)
    except Exception:
        return

    for model_name in model_names:
        if not model_name.startswith("waste_"):
            continue

        if class_name and not model_name.startswith("waste_{}_".format(class_name)):
            continue

        try:
            delete_srv(model_name)
            rospy.loginfo("[SPAWN REALISTIC ASSET] Deleted old model: %s", model_name)
        except Exception:
            pass


def main() -> None:
    rospy.init_node("spawn_realistic_asset", anonymous=True)

    class_name = clean(rospy.get_param("~class_name", "plastic")).lower()
    asset_folder = clean(rospy.get_param("~asset_folder", ""))
    model_index = int(rospy.get_param("~model_index", 1))

    x = float(rospy.get_param("~x", DEFAULT_X))
    y = float(rospy.get_param("~y", DEFAULT_Y))
    z = float(rospy.get_param("~z", DEFAULT_Z))
    yaw = float(rospy.get_param("~yaw", 0.0))

    random_asset = bool(rospy.get_param("~random_asset", True))
    random_y = bool(rospy.get_param("~random_y", False))
    random_yaw = bool(rospy.get_param("~random_yaw", False))

    force_on_belt = bool(rospy.get_param("~force_on_belt", True))
    belt_y_min = float(rospy.get_param("~belt_y_min", DEFAULT_BELT_Y_MIN))
    belt_y_max = float(rospy.get_param("~belt_y_max", DEFAULT_BELT_Y_MAX))

    static_model = bool(rospy.get_param("~static_model", True))

    delete_existing_same_name = bool(rospy.get_param("~delete_existing_same_name", True))
    delete_existing_waste = bool(rospy.get_param("~delete_existing_waste", False))
    delete_existing_same_class = bool(rospy.get_param("~delete_existing_same_class", False))

    if class_name not in VALID_CLASSES:
        rospy.logerr("[SPAWN REALISTIC ASSET] Invalid class_name=%s", class_name)
        rospy.logerr("[SPAWN REALISTIC ASSET] Valid classes: %s", VALID_CLASSES)
        sys.exit(1)

    package_root = package_root_from_script()

    class_dir = os.path.join(
        package_root,
        "models",
        "realistic_assets",
        class_name,
    )

    available_assets = list_asset_folders(class_dir)

    if not available_assets:
        rospy.logerr("[SPAWN REALISTIC ASSET] No OBJ-prepared assets found for class=%s", class_name)
        rospy.logerr("[SPAWN REALISTIC ASSET] class_dir=%s", class_dir)
        rospy.logerr("[SPAWN REALISTIC ASSET] Run prepare_realistic_assets.py --force first.")
        sys.exit(1)

    if asset_folder:
        if asset_folder not in available_assets:
            rospy.logerr("[SPAWN REALISTIC ASSET] Requested asset_folder not found: %s", asset_folder)
            rospy.logerr("[SPAWN REALISTIC ASSET] Available: %s", available_assets)
            sys.exit(1)
        selected_asset = asset_folder
    else:
        selected_asset = random.choice(available_assets) if random_asset else available_assets[0]

    asset_dir = os.path.join(class_dir, selected_asset)
    gazebo_model_name = "waste_{}_{:03d}".format(class_name, model_index)

    if random_y:
        y = random.uniform(belt_y_min, belt_y_max)

    if force_on_belt:
        y = clamp(y, belt_y_min, belt_y_max)

    if random_yaw:
        yaw = random.uniform(-math.pi, math.pi)

    metadata = load_metadata(asset_dir)
    asset_name = clean(metadata.get("asset_name", selected_asset))
    uid = clean(metadata.get("uid", ""))

    try:
        sdf_xml = load_sdf(asset_dir)
        sdf_xml = patch_sdf(
            sdf_xml=sdf_xml,
            asset_dir=asset_dir,
            static_model=static_model,
        )
    except Exception as exc:
        rospy.logerr("[SPAWN REALISTIC ASSET] Could not prepare SDF: %s", str(exc))
        rospy.logerr(traceback.format_exc())
        sys.exit(1)

    rospy.loginfo("[SPAWN REALISTIC ASSET] Waiting for Gazebo services...")
    rospy.wait_for_service("/gazebo/spawn_sdf_model")
    rospy.wait_for_service("/gazebo/delete_model")
    rospy.wait_for_service("/gazebo/get_world_properties")

    spawn_srv = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
    delete_srv = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
    world_props_srv = rospy.ServiceProxy("/gazebo/get_world_properties", GetWorldProperties)

    if delete_existing_waste:
        delete_existing_waste_models(world_props_srv, delete_srv, class_name=None)
    elif delete_existing_same_class:
        delete_existing_waste_models(world_props_srv, delete_srv, class_name=class_name)
    elif delete_existing_same_name:
        delete_model_if_exists(delete_srv, gazebo_model_name)

    pose = Pose()
    pose.position = Point(x, y, z)
    pose.orientation = yaw_to_quaternion(yaw)

    try:
        response = spawn_srv(
            gazebo_model_name,
            sdf_xml,
            "",
            pose,
            "world",
        )
    except Exception as exc:
        rospy.logerr("[SPAWN REALISTIC ASSET] Spawn service failed: %s", str(exc))
        rospy.logerr(traceback.format_exc())
        sys.exit(1)

    if not response.success:
        rospy.logerr("[SPAWN REALISTIC ASSET] Gazebo rejected spawn: %s", response.status_message)
        sys.exit(1)

    rospy.loginfo(
        "[SPAWN REALISTIC ASSET] Spawned model=%s class=%s asset=%s uid=%s asset_name=%s pose=(%.3f, %.3f, %.3f) yaw=%.3f static=%s",
        gazebo_model_name,
        class_name,
        selected_asset,
        uid,
        asset_name,
        x,
        y,
        z,
        yaw,
        static_model,
    )


if __name__ == "__main__":
    main()

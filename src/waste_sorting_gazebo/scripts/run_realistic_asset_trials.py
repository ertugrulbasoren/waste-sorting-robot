#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Run randomized realistic 3D asset sorting trials.

This node tests the Gazebo sorting pipeline using prepared Objaverse assets.

Pipeline:
  prepared realistic asset
      ↓
  spawn into Gazebo
      ↓
  publish /waste/pick_event
      ↓
  sort_executor moves object to correct bin
      ↓
  final model pose is checked
      ↓
  CSV trial log is written

This is not a YOLO evaluation node.
This is a realistic 3D asset sorting / executor / Gazebo integration test.

Expected prepared assets:
  src/waste_sorting_gazebo/models/realistic_assets/<class>/<asset_folder>/
    ├── model.sdf
    ├── model.config
    ├── meshes/model.obj
    ├── metadata.json
    └── bbox.json
"""

import csv
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
from gazebo_msgs.srv import DeleteModel, GetModelState, GetWorldProperties, SetModelState, SpawnModel
from geometry_msgs.msg import Point, Pose, Quaternion
from std_msgs.msg import String


VALID_CLASSES = ["glass", "metal", "paper", "plastic"]

# Final mapping:
# metal   -> red bin
# paper   -> blue bin
# glass   -> green bin
# plastic -> yellow bin
BIN_POSITIONS = {
    "metal": (1.5, -0.90, 0.20),
    "paper": (1.5, -0.45, 0.20),
    "glass": (1.5, 0.00, 0.20),
    "plastic": (1.5, 0.45, 0.20),
}

SPAWN_X_DEFAULT = -0.65
SPAWN_Y_DEFAULT = 0.0
SPAWN_Z_DEFAULT = 0.20

BELT_Y_MIN_DEFAULT = -0.04
BELT_Y_MAX_DEFAULT = 0.04


def package_root_from_script() -> str:
    return os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))


def project_root_from_script() -> str:
    return os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))


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
    assets: Dict[str, List[str]] = {class_name: [] for class_name in VALID_CLASSES}

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

            model_sdf = os.path.join(asset_dir, "model.sdf")
            model_obj = os.path.join(asset_dir, "meshes", "model.obj")

            if os.path.exists(model_sdf) and os.path.exists(model_obj):
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


def spawn_asset(
    spawn_srv,
    asset_dir: str,
    model_name: str,
    x: float,
    y: float,
    z: float,
    yaw: float,
    static_model: bool,
) -> bool:
    sdf_xml = load_sdf(asset_dir)
    sdf_xml = patch_sdf_for_spawn(sdf_xml, asset_dir, static_model)

    pose = Pose()
    pose.position = Point(x, y, z)
    pose.orientation = yaw_to_quaternion(yaw)

    response = spawn_srv(
        model_name,
        sdf_xml,
        "",
        pose,
        "world",
    )

    return bool(response.success)


def publish_pick_event(
    pub,
    track_id: int,
    class_name: str,
    model_name: str,
    confidence: float,
) -> None:
    payload = {
        "event_type": "pick_event",
        "track_id": int(track_id),
        "class_name": class_name,
        "model_name": model_name,
        "confidence": float(confidence),
        "center_x": 320,
        "center_y": 260,
        "pick_line_y": 330.0,
        "stamp": time.time(),
        "state": "CONFIRMED",
        "source": "run_realistic_asset_trials",
    }

    msg = String()
    msg.data = json.dumps(payload)

    pub.publish(msg)


def get_model_pose(get_model_state_srv, model_name: str) -> Tuple[bool, float, float, float]:
    try:
        response = get_model_state_srv(model_name, "world")

        if not response.success:
            return False, 0.0, 0.0, 0.0

        return (
            True,
            float(response.pose.position.x),
            float(response.pose.position.y),
            float(response.pose.position.z),
        )

    except Exception:
        return False, 0.0, 0.0, 0.0


def check_sorted_correct(class_name: str, x: float, y: float, tolerance: float) -> bool:
    expected_x, expected_y, _ = BIN_POSITIONS[class_name]

    dx = abs(x - expected_x)
    dy = abs(y - expected_y)

    return dx <= tolerance and dy <= tolerance


def ensure_log_file(path: str) -> None:
    os.makedirs(os.path.dirname(path), exist_ok=True)

    if os.path.exists(path):
        return

    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(
            [
                "wall_time",
                "trial_id",
                "class_name",
                "asset_folder",
                "asset_name",
                "uid",
                "model_name",
                "spawn_x",
                "spawn_y",
                "spawn_z",
                "expected_bin_x",
                "expected_bin_y",
                "expected_bin_z",
                "final_x",
                "final_y",
                "final_z",
                "spawn_success",
                "model_state_success",
                "sorted_correct",
            ]
        )


def append_log(
    path: str,
    trial_id: int,
    class_name: str,
    asset_folder: str,
    asset_name: str,
    uid: str,
    model_name: str,
    spawn_x: float,
    spawn_y: float,
    spawn_z: float,
    final_x: float,
    final_y: float,
    final_z: float,
    spawn_success: bool,
    model_state_success: bool,
    sorted_correct: bool,
) -> None:
    expected_x, expected_y, expected_z = BIN_POSITIONS[class_name]

    with open(path, "a", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(
            [
                round(time.time(), 3),
                trial_id,
                class_name,
                asset_folder,
                asset_name,
                uid,
                model_name,
                round(spawn_x, 4),
                round(spawn_y, 4),
                round(spawn_z, 4),
                expected_x,
                expected_y,
                expected_z,
                round(final_x, 4),
                round(final_y, 4),
                round(final_z, 4),
                spawn_success,
                model_state_success,
                sorted_correct,
            ]
        )


def choose_class(trial_id: int, mode: str) -> str:
    if mode == "balanced":
        return VALID_CLASSES[(trial_id - 1) % len(VALID_CLASSES)]

    return random.choice(VALID_CLASSES)


def main() -> None:
    rospy.init_node("run_realistic_asset_trials", anonymous=False)

    package_root = package_root_from_script()
    project_root = project_root_from_script()

    realistic_root = os.path.join(
        package_root,
        "models",
        "realistic_assets",
    )

    trial_count = int(rospy.get_param("~trial_count", 20))
    class_mode = clean(rospy.get_param("~class_mode", "balanced")).lower()
    static_model = bool(rospy.get_param("~static_model", True))
    delete_before_each_trial = bool(rospy.get_param("~delete_before_each_trial", True))
    delete_after_each_trial = bool(rospy.get_param("~delete_after_each_trial", False))

    spawn_x = float(rospy.get_param("~spawn_x", SPAWN_X_DEFAULT))
    spawn_z = float(rospy.get_param("~spawn_z", SPAWN_Z_DEFAULT))
    belt_y_min = float(rospy.get_param("~belt_y_min", BELT_Y_MIN_DEFAULT))
    belt_y_max = float(rospy.get_param("~belt_y_max", BELT_Y_MAX_DEFAULT))

    random_y = bool(rospy.get_param("~random_y", True))
    random_yaw = bool(rospy.get_param("~random_yaw", True))

    event_delay_sec = float(rospy.get_param("~event_delay_sec", 0.50))
    sort_wait_sec = float(rospy.get_param("~sort_wait_sec", 0.80))
    final_tolerance = float(rospy.get_param("~final_tolerance", 0.08))

    pick_event_topic = clean(rospy.get_param("~pick_event_topic", "/waste/pick_event"))

    log_path = clean(
        rospy.get_param(
            "~log_path",
            os.path.join(project_root, "logs", "trials", "realistic_asset_trials.csv"),
        )
    )

    assets = list_prepared_assets(realistic_root)

    rospy.loginfo("=" * 90)
    rospy.loginfo("[REALISTIC TRIALS] Started")
    rospy.loginfo("[REALISTIC TRIALS] realistic_root: %s", realistic_root)
    rospy.loginfo("[REALISTIC TRIALS] trial_count: %d", trial_count)
    rospy.loginfo("[REALISTIC TRIALS] class_mode: %s", class_mode)
    rospy.loginfo("[REALISTIC TRIALS] static_model: %s", static_model)
    rospy.loginfo("[REALISTIC TRIALS] log_path: %s", log_path)
    rospy.loginfo("[REALISTIC TRIALS] pick_event_topic: %s", pick_event_topic)

    for class_name in VALID_CLASSES:
        rospy.loginfo("[REALISTIC TRIALS] prepared %s assets: %d", class_name, len(assets[class_name]))

        if not assets[class_name]:
            rospy.logerr("[REALISTIC TRIALS] No prepared assets found for class=%s", class_name)
            sys.exit(1)

    rospy.loginfo("=" * 90)

    ensure_log_file(log_path)

    rospy.loginfo("[REALISTIC TRIALS] Waiting for Gazebo services...")
    rospy.wait_for_service("/gazebo/spawn_sdf_model")
    rospy.wait_for_service("/gazebo/delete_model")
    rospy.wait_for_service("/gazebo/get_world_properties")
    rospy.wait_for_service("/gazebo/get_model_state")

    spawn_srv = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
    delete_srv = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
    world_srv = rospy.ServiceProxy("/gazebo/get_world_properties", GetWorldProperties)
    get_model_state_srv = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)

    pick_pub = rospy.Publisher(pick_event_topic, String, queue_size=10)

    # Publisher connection time.
    time.sleep(1.0)

    total_success = 0
    total_spawn_fail = 0
    total_state_fail = 0

    for trial_id in range(1, trial_count + 1):
        if rospy.is_shutdown():
            break

        class_name = choose_class(trial_id, class_mode)

        if not assets[class_name]:
            rospy.logwarn("[REALISTIC TRIALS] No assets for class=%s, skipping trial", class_name)
            continue

        asset_folder = random.choice(assets[class_name])
        asset_dir = os.path.join(realistic_root, class_name, asset_folder)

        metadata = load_json(os.path.join(asset_dir, "metadata.json"))
        asset_name = clean(metadata.get("asset_name", asset_folder))
        uid = clean(metadata.get("uid", ""))

        model_name = "waste_{}_{:03d}".format(class_name, trial_id)

        y = random.uniform(belt_y_min, belt_y_max) if random_y else 0.0
        yaw = random.uniform(-math.pi, math.pi) if random_yaw else 0.0

        if delete_before_each_trial:
            delete_all_waste_models(world_srv, delete_srv)
            time.sleep(0.15)

        rospy.loginfo(
            "[REALISTIC TRIALS] Trial %d/%d class=%s asset=%s model=%s",
            trial_id,
            trial_count,
            class_name,
            asset_folder,
            model_name,
        )

        spawn_success = False
        model_state_success = False
        sorted_correct = False
        final_x = 0.0
        final_y = 0.0
        final_z = 0.0

        try:
            spawn_success = spawn_asset(
                spawn_srv=spawn_srv,
                asset_dir=asset_dir,
                model_name=model_name,
                x=spawn_x,
                y=y,
                z=spawn_z,
                yaw=yaw,
                static_model=static_model,
            )
        except Exception as exc:
            rospy.logerr("[REALISTIC TRIALS] Spawn failed trial=%d error=%s", trial_id, str(exc))
            rospy.logerr(traceback.format_exc())
            spawn_success = False

        if not spawn_success:
            total_spawn_fail += 1

            append_log(
                path=log_path,
                trial_id=trial_id,
                class_name=class_name,
                asset_folder=asset_folder,
                asset_name=asset_name,
                uid=uid,
                model_name=model_name,
                spawn_x=spawn_x,
                spawn_y=y,
                spawn_z=spawn_z,
                final_x=final_x,
                final_y=final_y,
                final_z=final_z,
                spawn_success=spawn_success,
                model_state_success=model_state_success,
                sorted_correct=sorted_correct,
            )

            continue

        time.sleep(event_delay_sec)

        publish_pick_event(
            pub=pick_pub,
            track_id=100000 + trial_id,
            class_name=class_name,
            model_name=model_name,
            confidence=0.99,
        )

        time.sleep(sort_wait_sec)

        model_state_success, final_x, final_y, final_z = get_model_pose(
            get_model_state_srv,
            model_name,
        )

        if not model_state_success:
            total_state_fail += 1
        else:
            sorted_correct = check_sorted_correct(
                class_name=class_name,
                x=final_x,
                y=final_y,
                tolerance=final_tolerance,
            )

        if sorted_correct:
            total_success += 1

        append_log(
            path=log_path,
            trial_id=trial_id,
            class_name=class_name,
            asset_folder=asset_folder,
            asset_name=asset_name,
            uid=uid,
            model_name=model_name,
            spawn_x=spawn_x,
            spawn_y=y,
            spawn_z=spawn_z,
            final_x=final_x,
            final_y=final_y,
            final_z=final_z,
            spawn_success=spawn_success,
            model_state_success=model_state_success,
            sorted_correct=sorted_correct,
        )

        rospy.loginfo(
            "[REALISTIC TRIALS] Result trial=%d class=%s final=(%.3f, %.3f, %.3f) correct=%s",
            trial_id,
            class_name,
            final_x,
            final_y,
            final_z,
            sorted_correct,
        )

        if delete_after_each_trial:
            delete_model_safe(delete_srv, model_name)
            time.sleep(0.10)

    rospy.loginfo("=" * 90)
    rospy.loginfo("[REALISTIC TRIALS] Finished")
    rospy.loginfo("[REALISTIC TRIALS] total_trials: %d", trial_count)
    rospy.loginfo("[REALISTIC TRIALS] sorted_correct: %d/%d", total_success, trial_count)
    rospy.loginfo("[REALISTIC TRIALS] spawn_fail: %d", total_spawn_fail)
    rospy.loginfo("[REALISTIC TRIALS] model_state_fail: %d", total_state_fail)
    rospy.loginfo("[REALISTIC TRIALS] log_path: %s", log_path)
    rospy.loginfo("=" * 90)


if __name__ == "__main__":
    main()

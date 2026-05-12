#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Spawn realistic procedural 3D waste models into Gazebo.

This script spawns one of the generated realistic waste models:

  class_name=plastic -> waste_plastic_deformed_bottle
  class_name=metal   -> waste_metal_crushed_can
  class_name=paper   -> waste_paper_crumpled
  class_name=glass   -> waste_glass_shard

Example usage:

  rosrun waste_sorting_gazebo spawn_realistic_waste.py _class_name:=plastic _model_index:=1
  rosrun waste_sorting_gazebo spawn_realistic_waste.py _class_name:=metal _model_index:=1
  rosrun waste_sorting_gazebo spawn_realistic_waste.py _class_name:=paper _model_index:=1
  rosrun waste_sorting_gazebo spawn_realistic_waste.py _class_name:=glass _model_index:=1

Optional parameters:

  _x:=-0.45
  _y:=0.0
  _z:=0.22
  _yaw:=0.0
  _random_y:=true
  _delete_existing:=true

Spawned Gazebo model names:

  waste_plastic_001
  waste_metal_001
  waste_paper_001
  waste_glass_001

These names are intentionally compatible with sort_executor.py.
"""

import os
import random
import sys
import traceback
from typing import Dict

import rospy
from gazebo_msgs.srv import DeleteModel, SpawnModel
from geometry_msgs.msg import Point, Pose, Quaternion


VALID_CLASSES = {"glass", "metal", "paper", "plastic"}


CLASS_TO_MODEL_FOLDER: Dict[str, str] = {
    "plastic": "waste_plastic_deformed_bottle",
    "metal": "waste_metal_crushed_can",
    "paper": "waste_paper_crumpled",
    "glass": "waste_glass_shard",
}


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = float(__import__("math").sin(yaw / 2.0))
    q.w = float(__import__("math").cos(yaw / 2.0))
    return q


def load_model_sdf(model_folder: str) -> str:
    package_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    model_sdf_path = os.path.join(
        package_root,
        "models",
        "realistic_waste",
        model_folder,
        "model.sdf",
    )

    if not os.path.exists(model_sdf_path):
        raise FileNotFoundError(
            "model.sdf not found: {}\n"
            "Run: rosrun waste_sorting_gazebo generate_realistic_waste_meshes.py".format(
                model_sdf_path
            )
        )

    with open(model_sdf_path, "r") as f:
        return f.read()


def main() -> None:
    rospy.init_node("spawn_realistic_waste", anonymous=True)

    class_name = str(rospy.get_param("~class_name", "plastic")).strip().lower()
    model_index = int(rospy.get_param("~model_index", 1))

    x = float(rospy.get_param("~x", -0.45))
    y = float(rospy.get_param("~y", 0.0))
    z = float(rospy.get_param("~z", 0.22))
    yaw = float(rospy.get_param("~yaw", 0.0))

    random_y = bool(rospy.get_param("~random_y", False))
    delete_existing = bool(rospy.get_param("~delete_existing", True))

    if class_name not in VALID_CLASSES:
        rospy.logerr("[SPAWN REALISTIC WASTE] Unsupported class_name=%s", class_name)
        rospy.logerr("[SPAWN REALISTIC WASTE] Valid classes: %s", sorted(list(VALID_CLASSES)))
        sys.exit(1)

    if random_y:
        y = random.uniform(-0.12, 0.12)

    if yaw == 0.0:
        yaw = random.uniform(-3.14159, 3.14159)

    source_model_folder = CLASS_TO_MODEL_FOLDER[class_name]
    gazebo_model_name = "waste_{}_{:03d}".format(class_name, model_index)

    rospy.loginfo("[SPAWN REALISTIC WASTE] Waiting for Gazebo services...")
    rospy.wait_for_service("/gazebo/spawn_sdf_model")
    rospy.wait_for_service("/gazebo/delete_model")

    spawn_srv = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
    delete_srv = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)

    if delete_existing:
        try:
            delete_srv(gazebo_model_name)
        except Exception:
            pass

    try:
        sdf_xml = load_model_sdf(source_model_folder)
    except Exception as exc:
        rospy.logerr("[SPAWN REALISTIC WASTE] Could not load SDF: %s", str(exc))
        rospy.logerr(traceback.format_exc())
        sys.exit(1)

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
        rospy.logerr("[SPAWN REALISTIC WASTE] Spawn service failed: %s", str(exc))
        rospy.logerr(traceback.format_exc())
        sys.exit(1)

    if not response.success:
        rospy.logerr("[SPAWN REALISTIC WASTE] Gazebo rejected spawn: %s", response.status_message)
        sys.exit(1)

    rospy.loginfo(
        "[SPAWN REALISTIC WASTE] Spawned %s from %s at x=%.2f y=%.2f z=%.2f yaw=%.2f",
        gazebo_model_name,
        source_model_folder,
        x,
        y,
        z,
        yaw,
    )


if __name__ == "__main__":
    main()

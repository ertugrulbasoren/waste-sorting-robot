#!/usr/bin/env python3
import os
import time
import rospy
import rospkg
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose


CURRENT_MODEL_FILE = os.path.expanduser("~/waste_sorting_ws/current_waste_model.txt")


def get_model_path(waste_type):
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path("waste_sorting_gazebo")

    model_map = {
        "plastic": "trash_object_red",
        "metal": "trash_object_yellow",
        "paper": "trash_object_green"
    }

    if waste_type not in model_map:
        return None, None

    model_dir = model_map[waste_type]
    model_path = os.path.join(pkg_path, "models", model_dir, "model.sdf")

    # HER SEFERİNDE BENZERSİZ İSİM
    model_name = "waste_{}_{}".format(waste_type, int(time.time() * 1000))

    return model_path, model_name


def spawn_model(model_path, model_name):
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    spawn = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)

    with open(model_path, "r") as f:
        model_xml = f.read()

    pose = Pose()
    pose.position.x = -0.85
    pose.position.y = 0.0
    pose.position.z = 0.25
    pose.orientation.x = 0.0
    pose.orientation.y = 0.0
    pose.orientation.z = 0.0
    pose.orientation.w = 1.0

    spawn(model_name, model_xml, "", pose, "world")

    with open(CURRENT_MODEL_FILE, "w", encoding="utf-8") as f:
        f.write(model_name)

    rospy.loginfo("Spawned: %s", model_name)


def main():
    rospy.init_node("spawn_trash")

    waste_type = rospy.get_param("~waste_type", "plastic").strip().lower()

    model_path, model_name = get_model_path(waste_type)
    if model_path is None:
        rospy.logerr("Unknown waste type: %s", waste_type)
        return

    spawn_model(model_path, model_name)


if __name__ == "__main__":
    main()

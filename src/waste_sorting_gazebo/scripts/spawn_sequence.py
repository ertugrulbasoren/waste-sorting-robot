#!/usr/bin/env python3
import os
import rospy
import subprocess
import rospkg
from gazebo_msgs.srv import DeleteModel


TYPES = ["plastic", "metal", "paper"]


def get_state_file():
    rospack = rospkg.RosPack()
    gazebo_pkg_path = rospack.get_path("waste_sorting_gazebo")
    repo_root = os.path.abspath(os.path.join(gazebo_pkg_path, "..", ".."))
    return os.path.join(repo_root, "sequence_state.txt")


def read_index(state_file):
    if not os.path.exists(state_file):
        return 0

    try:
        with open(state_file, "r") as f:
            return int(f.read().strip())
    except Exception:
        return 0


def write_index(state_file, index):
    with open(state_file, "w") as f:
        f.write(str(index))


if __name__ == "__main__":
    rospy.init_node("spawn_sequence")

    state_file = get_state_file()

    rospy.wait_for_service("/gazebo/delete_model")
    delete_model = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)

    for name in ["waste_plastic", "waste_metal", "trash_red", "trash_green", "trash_yellow"]:
        try:
            delete_model(name)
            rospy.loginfo("Deleted existing model: %s", name)
        except Exception:
            pass

    idx = read_index(state_file)
    waste_type = TYPES[idx % len(TYPES)]
    write_index(state_file, (idx + 1) % len(TYPES))

    subprocess.call([
        "rosrun",
        "waste_sorting_gazebo",
        "spawn_trash.py",
        "_waste_type:=" + waste_type
    ])

    rospy.loginfo("Spawned next object: %s", waste_type)

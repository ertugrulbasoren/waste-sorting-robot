#!/usr/bin/env python3
import os
import rospy
import subprocess
from gazebo_msgs.srv import DeleteModel

STATE_FILE = os.path.expanduser("~/waste_sorting_ws/sequence_state.txt")
TYPES = ["plastic", "metal", "paper"]

def read_index():
    if not os.path.exists(STATE_FILE):
        return 0
    try:
        with open(STATE_FILE, "r") as f:
            return int(f.read().strip())
    except Exception:
        return 0

def write_index(i):
    with open(STATE_FILE, "w") as f:
        f.write(str(i))

if __name__ == "__main__":
    rospy.init_node("spawn_sequence")

    rospy.wait_for_service("/gazebo/delete_model")
    delete_model = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)

    for name in ["waste_plastic", "waste_metal", "trash_red", "trash_green", "trash_yellow"]:
        try:
            delete_model(name)
            rospy.loginfo("Deleted existing model: %s", name)
        except Exception:
            pass

    idx = read_index()
    waste_type = TYPES[idx % len(TYPES)]
    write_index((idx + 1) % len(TYPES))

    subprocess.call([
        "rosrun",
        "waste_sorting_gazebo",
        "spawn_trash.py",
        "_waste_type:=" + waste_type
    ])

    rospy.loginfo("Spawned next object: %s", waste_type)

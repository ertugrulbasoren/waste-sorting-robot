#!/usr/bin/env python3
print("SPAWN SEQUENCE STARTED")
import os
import rospy
import subprocess
from gazebo_msgs.srv import DeleteModel

STATE_FILE = os.path.expanduser("~/waste_sorting_ws/sequence_state.txt")
COLORS = ["red", "green", "yellow"]

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

    for name in ["trash_red", "trash_green", "trash_yellow"]:
        try:
            delete_model(name)
            rospy.loginfo("Deleted existing model: %s", name)
        except Exception:
            pass

    idx = read_index()
    color = COLORS[idx % len(COLORS)]
    write_index((idx + 1) % len(COLORS))

    subprocess.call([
        "rosrun",
        "waste_sorting_gazebo",
        "spawn_trash.py",
        "_color:=" + color
    ])

    rospy.loginfo("Spawned next object: %s", color)

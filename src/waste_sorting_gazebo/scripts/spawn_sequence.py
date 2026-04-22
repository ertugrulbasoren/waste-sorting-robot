#!/usr/bin/env python3

import os
import subprocess
import time
import rospy

STATE_FILE = os.path.expanduser("~/waste_sorting_ws/sequence_state.txt")
CLASS_SEQUENCE = ["plastic", "paper", "metal"]


def read_index():
    if not os.path.exists(STATE_FILE):
        return 0

    try:
        with open(STATE_FILE, "r", encoding="utf-8") as f:
            return int(f.read().strip())
    except Exception:
        return 0


def write_index(i):
    with open(STATE_FILE, "w", encoding="utf-8") as f:
        f.write(str(i))


def main():
    rospy.init_node("spawn_sequence")
    rospy.loginfo("SPAWN SEQUENCE STARTED")

    # İlk açılışta tüm node'ların hazır olması için bekle
    initial_delay = float(rospy.get_param("~initial_delay", 4.0))
    rospy.loginfo("Waiting %.1f seconds before spawning...", initial_delay)
    time.sleep(initial_delay)

    idx = read_index()
    waste_type = CLASS_SEQUENCE[idx % len(CLASS_SEQUENCE)]
    write_index((idx + 1) % len(CLASS_SEQUENCE))

    cmd = [
        "rosrun",
        "waste_sorting_gazebo",
        "spawn_trash.py",
        "_waste_type:={}".format(waste_type),
    ]

    result = subprocess.call(cmd)
    if result != 0:
        rospy.logerr("spawn_trash.py failed with exit code %s", result)
        return

    rospy.loginfo("Spawned next object type: %s", waste_type)


if __name__ == "__main__":
    main()

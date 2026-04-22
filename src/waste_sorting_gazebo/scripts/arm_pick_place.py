#!/usr/bin/env python3
import json
import threading
import time
import os
import subprocess
import math
import rospy

from std_msgs.msg import String
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState


class ArmPickPlace:
    def __init__(self):
        rospy.init_node("arm_pick_place")

        self.gripper_model = "simple_sort_gripper"
        self.current_model_file = os.path.expanduser("~/waste_sorting_ws/current_waste_model.txt")

        # Fake arm motion tuning
        self.step_dt = 0.03
        self.pick_attach_delay = 0.20
        self.place_release_delay = 0.20
        self.carry_z_offset = 0.10

        # Fixed demo pick zone on conveyor
        self.pick_pos = {"x": 0.62, "y": 0.00, "z": 0.24}
        self.pick_hover = {"x": 0.62, "y": 0.00, "z": 0.55}
        self.home_pos = {"x": 0.95, "y": -0.10, "z": 0.60}

        # Correct bin mapping
        self.place_pos = {
            "plastic": {"x": 1.38, "y": -0.42, "z": 0.30},
            "paper":   {"x": 1.38, "y":  0.00, "z": 0.30},
            "metal":   {"x": 1.38, "y":  0.42, "z": 0.30}
        }

        self.place_hover = {
            "plastic": {"x": 1.28, "y": -0.42, "z": 0.62},
            "paper":   {"x": 1.28, "y":  0.00, "z": 0.62},
            "metal":   {"x": 1.28, "y":  0.42, "z": 0.62}
        }

        self.busy = False
        self.lock = threading.Lock()

        rospy.wait_for_service("/gazebo/set_model_state")
        self.set_model_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)

        rospy.sleep(1.0)
        self.set_pose(self.gripper_model, self.home_pos["x"], self.home_pos["y"], self.home_pos["z"])

        self.pick_sub = rospy.Subscriber("/waste/pick_event", String, self.pick_callback, queue_size=10)

        rospy.loginfo("ARM PICK PLACE READY - smooth fake mode")

    def normalize_class_name(self, raw_name):
        raw_name = (raw_name or "").strip().lower()

        color_to_type = {
            "red": "plastic",
            "green": "paper",
            "yellow": "metal",
        }

        if raw_name in color_to_type:
            return color_to_type[raw_name]

        if raw_name in self.place_pos:
            return raw_name

        return None

    def get_current_model_name(self):
        if not os.path.exists(self.current_model_file):
            return None
        try:
            with open(self.current_model_file, "r", encoding="utf-8") as f:
                name = f.read().strip()
                return name if name else None
        except Exception:
            return None

    def pick_callback(self, msg):
        try:
            data = json.loads(msg.data)
        except Exception as e:
            rospy.logerr("pick_event json error: %s", str(e))
            return

        raw_class_name = data.get("class_name", "")
        class_name = self.normalize_class_name(raw_class_name)

        if class_name not in self.place_pos:
            rospy.logwarn("Unknown class_name: %s", raw_class_name)
            return

        model_name = self.get_current_model_name()
        if not model_name:
            rospy.logwarn("No current waste model name found")
            return

        with self.lock:
            if self.busy:
                rospy.logwarn("Arm busy, skipping event")
                return
            self.busy = True

        worker = threading.Thread(target=self.run_sequence, args=(class_name, model_name), daemon=True)
        worker.start()

    def run_sequence(self, class_name, model_name):
        try:
            # 1) Go from home to pick hover
            self.move_line(self.home_pos, self.pick_hover, 1.0)

            # 2) Descend to pick
            self.move_line(self.pick_hover, self.pick_pos, 0.7)

            # 3) Small grab delay
            rospy.sleep(self.pick_attach_delay)

            # Snap object only at the moment of "grasp"
            self.set_pose(model_name, self.pick_pos["x"], self.pick_pos["y"], self.pick_pos["z"] - self.carry_z_offset)

            # 4) Lift straight up with object
            self.move_line(self.pick_pos, self.pick_hover, 0.8, carry_model=model_name)

            # 5) Move through a higher midpoint to avoid harsh teleport feel
            mid_arc = {
                "x": (self.pick_hover["x"] + self.place_hover[class_name]["x"]) / 2.0,
                "y": (self.pick_hover["y"] + self.place_hover[class_name]["y"]) / 2.0,
                "z": 0.78
            }

            self.move_line(self.pick_hover, mid_arc, 1.0, carry_model=model_name)
            self.move_line(mid_arc, self.place_hover[class_name], 1.0, carry_model=model_name)

            # 6) Descend to place
            self.move_line(self.place_hover[class_name], self.place_pos[class_name], 0.8, carry_model=model_name)

            # 7) Release delay
            rospy.sleep(self.place_release_delay)

            # Leave object in bin
            self.set_pose(
                model_name,
                self.place_pos[class_name]["x"],
                self.place_pos[class_name]["y"],
                self.place_pos[class_name]["z"]
            )

            # 8) Retract upward
            self.move_line(self.place_pos[class_name], self.place_hover[class_name], 0.8)

            # 9) Return home through the same high arc idea
            back_mid = {
                "x": (self.place_hover[class_name]["x"] + self.home_pos["x"]) / 2.0,
                "y": (self.place_hover[class_name]["y"] + self.home_pos["y"]) / 2.0,
                "z": 0.78
            }

            self.move_line(self.place_hover[class_name], back_mid, 1.0)
            self.move_line(back_mid, self.home_pos, 1.0)

            rospy.loginfo("ARM SORT DONE: %s -> %s", class_name, model_name)

            subprocess.Popen([
                "rosrun",
                "waste_sorting_gazebo",
                "spawn_sequence.py"
            ])

        except Exception as e:
            rospy.logerr("Arm sequence failed: %s", str(e))

        with self.lock:
            self.busy = False

    def ease_in_out(self, a):
        # smoother than linear interpolation
        return 0.5 - 0.5 * math.cos(math.pi * a)

    def move_line(self, start_pos, end_pos, duration, carry_model=None):
        steps = max(1, int(duration / self.step_dt))

        for i in range(steps + 1):
            a = float(i) / float(steps)
            s = self.ease_in_out(a)

            gx = start_pos["x"] + (end_pos["x"] - start_pos["x"]) * s
            gy = start_pos["y"] + (end_pos["y"] - start_pos["y"]) * s
            gz = start_pos["z"] + (end_pos["z"] - start_pos["z"]) * s

            self.set_pose(self.gripper_model, gx, gy, gz)

            if carry_model is not None:
                self.set_pose(carry_model, gx, gy, gz - self.carry_z_offset)

            time.sleep(self.step_dt)

    def set_pose(self, model_name, x, y, z):
        state = ModelState()
        state.model_name = model_name
        state.reference_frame = "world"

        state.pose.position.x = x
        state.pose.position.y = y
        state.pose.position.z = z

        state.pose.orientation.x = 0.0
        state.pose.orientation.y = 0.0
        state.pose.orientation.z = 0.0
        state.pose.orientation.w = 1.0

        self.set_model_state(state)


if __name__ == "__main__":
    ArmPickPlace()
    rospy.spin()

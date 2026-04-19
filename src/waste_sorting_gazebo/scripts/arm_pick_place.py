#!/usr/bin/env python3
import json
import threading
import time
import rospy

from std_msgs.msg import String
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState


class ArmPickPlace:
    def __init__(self):
        rospy.init_node("arm_pick_place")

        self.gripper_model = "simple_sort_gripper"

        self.model_map = {
            "red": "trash_red",
            "green": "trash_green",
            "yellow": "trash_yellow"
        }

        self.pick_pos = {"x": 0.62, "y": 0.00, "z": 0.24}
        self.pick_hover = {"x": 0.62, "y": 0.00, "z": 0.55}
        self.lift_pos = {"x": 0.95, "y": -0.10, "z": 0.60}

        self.place_pos = {
            "red":    {"x": 1.50, "y": -0.45, "z": 0.42},
            "green":  {"x": 1.50, "y":  0.00, "z": 0.42},
            "yellow": {"x": 1.50, "y":  0.45, "z": 0.42}
        }

        self.place_hover = {
            "red":    {"x": 1.40, "y": -0.45, "z": 0.60},
            "green":  {"x": 1.40, "y":  0.00, "z": 0.60},
            "yellow": {"x": 1.40, "y":  0.45, "z": 0.60}
        }

        self.home_pos = {"x": 0.95, "y": -0.10, "z": 0.60}

        self.busy = False
        self.lock = threading.Lock()

        rospy.wait_for_service("/gazebo/set_model_state")
        self.set_model_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)

        rospy.sleep(1.0)
        self.set_pose(self.gripper_model, self.home_pos["x"], self.home_pos["y"], self.home_pos["z"])

        self.pick_sub = rospy.Subscriber("/waste/pick_event", String, self.pick_callback, queue_size=10)

        rospy.loginfo("ARM PICK PLACE READY")

    def pick_callback(self, msg):
        try:
            data = json.loads(msg.data)
        except Exception as e:
            rospy.logerr("pick_event json error: %s", str(e))
            return

        class_name = data.get("class_name", "").strip()
        if class_name not in self.model_map:
            rospy.logwarn("Unknown class_name: %s", class_name)
            return

        with self.lock:
            if self.busy:
                rospy.logwarn("Arm busy, skipping event")
                return
            self.busy = True

        worker = threading.Thread(target=self.run_sequence, args=(class_name,), daemon=True)
        worker.start()

    def run_sequence(self, class_name):
        cube_model = self.model_map[class_name]

        try:
            self.move_model(self.gripper_model, self.home_pos, self.pick_hover, 0.8, carry_model=None)
            self.move_model(self.gripper_model, self.pick_hover, self.pick_pos, 0.5, carry_model=None)

            self.set_pose(cube_model, self.pick_pos["x"], self.pick_pos["y"], self.pick_pos["z"])

            self.move_model(self.gripper_model, self.pick_pos, self.pick_hover, 0.5, carry_model=cube_model)
            self.move_model(self.gripper_model, self.pick_hover, self.lift_pos, 0.8, carry_model=cube_model)
            self.move_model(self.gripper_model, self.lift_pos, self.place_hover[class_name], 1.0, carry_model=cube_model)
            self.move_model(self.gripper_model, self.place_hover[class_name], self.place_pos[class_name], 0.5, carry_model=cube_model)

            self.set_pose(
                cube_model,
                self.place_pos[class_name]["x"],
                self.place_pos[class_name]["y"],
                self.place_pos[class_name]["z"]
            )

            rospy.sleep(0.2)

            self.move_model(self.gripper_model, self.place_pos[class_name], self.place_hover[class_name], 0.5, carry_model=None)
            self.move_model(self.gripper_model, self.place_hover[class_name], self.home_pos, 1.0, carry_model=None)

            rospy.loginfo("ARM SORT DONE: %s", class_name)

        except Exception as e:
            rospy.logerr("Arm sequence failed: %s", str(e))

        with self.lock:
            self.busy = False

    def move_model(self, gripper_model, start_pos, end_pos, duration, carry_model=None):
        steps = max(1, int(duration / 0.03))

        for i in range(steps + 1):
            a = float(i) / float(steps)

            gx = start_pos["x"] + (end_pos["x"] - start_pos["x"]) * a
            gy = start_pos["y"] + (end_pos["y"] - start_pos["y"]) * a
            gz = start_pos["z"] + (end_pos["z"] - start_pos["z"]) * a

            self.set_pose(gripper_model, gx, gy, gz)

            if carry_model is not None:
                self.set_pose(carry_model, gx, gy, gz - 0.10)

            time.sleep(0.03)

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

        state.twist.linear.x = 0.0
        state.twist.linear.y = 0.0
        state.twist.linear.z = 0.0
        state.twist.angular.x = 0.0
        state.twist.angular.y = 0.0
        state.twist.angular.z = 0.0

        self.set_model_state(state)


if __name__ == "__main__":
    ArmPickPlace()
    rospy.spin()

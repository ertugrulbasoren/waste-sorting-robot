#!/usr/bin/env python3
import json
import rospy

from std_msgs.msg import String
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState


class SortExecutor:
    def __init__(self):
        rospy.init_node("sort_executor")

        self.model_map = {
            "red": "trash_red",
            "green": "trash_green",
            "yellow": "trash_yellow"
        }

        self.bin_positions = {
            "red":    {"x": 1.50, "y": -0.45, "z": 0.35},
            "green":  {"x": 1.50, "y":  0.00, "z": 0.35},
            "yellow": {"x": 1.50, "y":  0.45, "z": 0.35}
        }

        rospy.wait_for_service("/gazebo/set_model_state")
        self.set_model_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)

        self.pick_sub = rospy.Subscriber("/waste/pick_event", String, self.pick_callback, queue_size=10)

        rospy.loginfo("SORT EXECUTOR READY")

    def pick_callback(self, msg):
        try:
            data = json.loads(msg.data)
        except Exception as e:
            rospy.logerr("JSON parse error: %s", str(e))
            return

        class_name = data.get("class_name", "").strip()
        if class_name not in self.model_map:
            rospy.logwarn("Unknown class_name: %s", class_name)
            return

        model_name = self.model_map[class_name]
        target = self.bin_positions[class_name]

        state = ModelState()
        state.model_name = model_name
        state.reference_frame = "world"

        state.pose.position.x = target["x"]
        state.pose.position.y = target["y"]
        state.pose.position.z = target["z"]

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

        try:
            self.set_model_state(state)
            rospy.loginfo("SORTED: %s -> bin %s", model_name, class_name)
        except Exception as e:
            rospy.logerr("SetModelState failed: %s", str(e))


if __name__ == "__main__":
    SortExecutor()
    rospy.spin()

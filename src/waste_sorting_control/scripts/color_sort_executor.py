#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState

class ColorSortExecutor:
    def __init__(self):
        rospy.init_node('color_sort_executor')

        self.sub = rospy.Subscriber('/detected_color', String, self.callback)

        self.last_action_time = rospy.Time.now()
        self.cooldown = rospy.Duration(3.0)

        rospy.wait_for_service('/gazebo/set_model_state')
        self.set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

    def move_model(self, model_name, x, y, z=0.15):
        state = ModelState()
        state.model_name = model_name
        state.pose.position.x = x
        state.pose.position.y = y
        state.pose.position.z = z
        state.pose.orientation.x = 0.0
        state.pose.orientation.y = 0.0
        state.pose.orientation.z = 0.0
        state.pose.orientation.w = 1.0
        state.reference_frame = "world"

        self.set_state(state)

    def callback(self, msg):
        now = rospy.Time.now()

        if now - self.last_action_time < self.cooldown:
            return

        color = msg.data

        try:
            if color == "red":
                self.move_model("trash_red",1.0, 0.9, 0.15)
                rospy.loginfo("Moved trash_red to BLUE bin")
            elif color == "green":
                self.move_model("trash_green", 1.45, 0.9, 0.15)
                rospy.loginfo("Moved trash_green to GREEN bin")
            elif color == "yellow":
                self.move_model("trash_yellow", 1.9, 0.9, 0.15)
                rospy.loginfo("Moved trash_yellow to YELLOW bin")
            else:
                rospy.logwarn("Unknown detected color: %s", color)
                return

            self.last_action_time = now

        except rospy.ServiceException as e:
            rospy.logerr("SetModelState failed: %s", str(e))

if __name__ == '__main__':
    ColorSortExecutor()
    rospy.spin()

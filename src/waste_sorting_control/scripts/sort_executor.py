#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState

class SortExecutor:
    def __init__(self):
        rospy.init_node('sort_executor')

        self.sub = rospy.Subscriber('/detected_object', Point, self.callback)
        self.last_action_time = rospy.Time.now()
        self.cooldown = rospy.Duration(3.0)

        rospy.wait_for_service('/gazebo/set_model_state')
        self.set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

    def callback(self, msg):
        now = rospy.Time.now()

        if now - self.last_action_time < self.cooldown:
            return

        x = msg.x
        area = msg.z

        if area < 20:
            return

        if x < 213:
            zone = "LEFT"
            target_x = 1.0
            target_y = 0.9
        elif x < 426:
            zone = "CENTER"
            target_x = 1.45
            target_y = 0.9
        else:
            zone = "RIGHT"
            target_x = 1.9
            target_y = 0.9

        state = ModelState()
        state.model_name = "trash1"
        state.pose.position.x = target_x
        state.pose.position.y = target_y
        state.pose.position.z = 0.35
        state.pose.orientation.x = 0.0
        state.pose.orientation.y = 0.0
        state.pose.orientation.z = 0.0
        state.pose.orientation.w = 1.0
        state.reference_frame = "world"

        try:
            self.set_state(state)
            rospy.loginfo("Moved trash1 to %s bin", zone)
            self.last_action_time = now
        except rospy.ServiceException as e:
            rospy.logerr("SetModelState failed: %s", str(e))

if __name__ == '__main__':
    SortExecutor()
    rospy.spin()

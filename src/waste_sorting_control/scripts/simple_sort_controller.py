#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point

class SimpleSortController:
    def __init__(self):
        rospy.init_node('simple_sort_controller')

        self.sub = rospy.Subscriber('/detected_object', Point, self.callback)
        self.last_zone = None

    def callback(self, msg):
        x = msg.x
        y = msg.y
        area = msg.z

        if area < 20:
            return

        if x < 213:
            zone = "LEFT"
        elif x < 426:
            zone = "CENTER"
        else:
            zone = "RIGHT"

        if zone != self.last_zone:
            rospy.loginfo("Object zone: %s | x=%.1f y=%.1f area=%.1f", zone, x, y, area)
            self.last_zone = zone

if __name__ == '__main__':
    SimpleSortController()
    rospy.spin()

#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import numpy as np

class RedDetector:
    def __init__(self):
        rospy.init_node('red_detector_node')

        self.bridge = CvBridge()

        self.image_sub = rospy.Subscriber('/top_camera/image_raw', Image, self.callback)
        self.object_pub = rospy.Publisher('/detected_object', Point, queue_size=10)

    def callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            rospy.logerr("CvBridge error: %s", str(e))
            return

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        lower_red1 = np.array([0, 120, 70])
        upper_red1 = np.array([10, 255, 255])

        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([180, 255, 255])

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = mask1 + mask2

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        largest_contour = None
        largest_area = 0

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 20 and area > largest_area:
                largest_area = area
                largest_contour = cnt

        if largest_contour is not None:
            x, y, w, h = cv2.boundingRect(largest_contour)

            center_x = x + w // 2
            center_y = y + h // 2

            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.circle(frame, (center_x, center_y), 5, (255, 0, 0), -1)

            text = f"RED x={center_x} y={center_y}"
            cv2.putText(frame, text, (x, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            point_msg = Point()
            point_msg.x = float(center_x)
            point_msg.y = float(center_y)
            point_msg.z = float(largest_area)

            self.object_pub.publish(point_msg)

            rospy.loginfo_throttle(
                1,
                "Detected object center: x=%.1f y=%.1f area=%.1f" %
                (point_msg.x, point_msg.y, point_msg.z)
            )

        cv2.imshow("Top Camera Detection", frame)
        cv2.imshow("Red Mask", mask)
        cv2.waitKey(1)

if __name__ == '__main__':
    RedDetector()
    rospy.spin()

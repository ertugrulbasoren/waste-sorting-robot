#!/usr/bin/env python3
import cv2
import rospy
import numpy as np

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from waste_sorting_gazebo.msg import Detection2D


class SimDetector:
    def __init__(self):
        rospy.init_node("sim_detector")

        self.bridge = CvBridge()

        self.sub = rospy.Subscriber(
            "/top_camera/image_raw",
            Image,
            self.callback,
            queue_size=1
        )

        self.pub = rospy.Publisher("/waste/detection", Detection2D, queue_size=10)
        self.img_pub = rospy.Publisher("/waste/detection_image", Image, queue_size=1)

        rospy.loginfo("SIM DETECTOR READY")

    def callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        _, thresh = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY_INV)

        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) == 0:
            self.publish_image(frame)
            return

        c = max(contours, key=cv2.contourArea)

        if cv2.contourArea(c) < 200:
            self.publish_image(frame)
            return

        x, y, w, h = cv2.boundingRect(c)

        cx = int(x + w / 2)
        cy = int(y + h / 2)

        det = Detection2D()
        det.class_name = "object"
        det.confidence = 1.0
        det.x_min = x
        det.y_min = y
        det.x_max = x + w
        det.y_max = y + h
        det.center_x = cx
        det.center_y = cy

        self.pub.publish(det)

        # DRAW
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.circle(frame, (cx, cy), 4, (0, 0, 255), -1)

        self.publish_image(frame)

    def publish_image(self, frame):
        msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        self.img_pub.publish(msg)


if __name__ == "__main__":
    SimDetector()
    rospy.spin()

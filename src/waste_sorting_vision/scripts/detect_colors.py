#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np

class ColorDetector:
    def __init__(self):
        rospy.init_node('color_detector_node')

        self.bridge = CvBridge()

        self.image_sub = rospy.Subscriber('/top_camera/image_raw', Image, self.callback)
        self.object_pub = rospy.Publisher('/detected_object', Point, queue_size=10)
        self.color_pub = rospy.Publisher('/detected_color', String, queue_size=10)

    def callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            rospy.logerr("CvBridge error: %s", str(e))
            return

        # Sadece spawn bölgesini işle
        # Sol taraf / konveyör başlangıcı
        roi_x1 = 0
        roi_y1 = 120
        roi_x2 = 320
        roi_y2 = 470

        roi = frame[roi_y1:roi_y2, roi_x1:roi_x2]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        color_ranges = {
            'red': [
                (np.array([0, 120, 70]), np.array([10, 255, 255])),
                (np.array([170, 120, 70]), np.array([180, 255, 255]))
            ],
            'green': [
                (np.array([40, 70, 70]), np.array([80, 255, 255]))
            ],
            'yellow': [
                (np.array([20, 100, 100]), np.array([35, 255, 255]))
            ]
        }

        best_contour = None
        best_area = 0
        best_color = None
        best_bbox = None

        mask_display = np.zeros(roi.shape[:2], dtype=np.uint8)

        for color_name, ranges in color_ranges.items():
            mask = np.zeros(roi.shape[:2], dtype=np.uint8)

            for lower, upper in ranges:
                mask += cv2.inRange(hsv, lower, upper)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for cnt in contours:
                area = cv2.contourArea(cnt)

                # Küçük gürültüleri ele
                if area > 20 and area > best_area:
                    x, y, w, h = cv2.boundingRect(cnt)

                    best_area = area
                    best_contour = cnt
                    best_color = color_name

                    # ROI içindeki koordinatı tam frame koordinatına çevir
                    best_bbox = (x + roi_x1, y + roi_y1, w, h)
                    mask_display = mask.copy()

        # Hangi alanın işlendiğini ekranda göster
        cv2.rectangle(frame, (roi_x1, roi_y1), (roi_x2, roi_y2), (255, 255, 255), 2)

        if best_contour is not None:
            x, y, w, h = best_bbox

            center_x = x + w // 2
            center_y = y + h // 2

            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.circle(frame, (center_x, center_y), 5, (255, 0, 0), -1)

            label = f"{best_color.upper()} x={center_x} y={center_y}"
            cv2.putText(
                frame,
                label,
                (x, y - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                2
            )

            point_msg = Point()
            point_msg.x = float(center_x)
            point_msg.y = float(center_y)
            point_msg.z = float(best_area)

            color_msg = String()
            color_msg.data = best_color

            self.object_pub.publish(point_msg)
            self.color_pub.publish(color_msg)

            rospy.loginfo_throttle(
                1,
                "Detected color=%s x=%.1f y=%.1f area=%.1f" %
                (best_color, point_msg.x, point_msg.y, point_msg.z)
            )

        cv2.imshow("Top Camera Detection", frame)
        cv2.imshow("Color Mask", mask_display)
        cv2.waitKey(1)

if __name__ == '__main__':
    ColorDetector()
    rospy.spin()

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

        self.image_sub = rospy.Subscriber(
            "/top_camera/image_raw",
            Image,
            self.callback,
            queue_size=1
        )

        self.det_pub = rospy.Publisher("/waste/detection", Detection2D, queue_size=10)
        self.img_pub = rospy.Publisher("/waste/detection_image", Image, queue_size=1)

        rospy.loginfo("SIM DETECTOR READY")

    def callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr("cv_bridge error: %s", str(e))
            return

        annotated = frame.copy()
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # RED mask (2 range)
        red_lower1 = np.array([0, 80, 80], dtype=np.uint8)
        red_upper1 = np.array([10, 255, 255], dtype=np.uint8)
        red_lower2 = np.array([170, 80, 80], dtype=np.uint8)
        red_upper2 = np.array([180, 255, 255], dtype=np.uint8)

        red_mask1 = cv2.inRange(hsv, red_lower1, red_upper1)
        red_mask2 = cv2.inRange(hsv, red_lower2, red_upper2)
        red_mask = cv2.bitwise_or(red_mask1, red_mask2)

        # GREEN mask
        green_lower = np.array([35, 60, 60], dtype=np.uint8)
        green_upper = np.array([90, 255, 255], dtype=np.uint8)
        green_mask = cv2.inRange(hsv, green_lower, green_upper)

        # YELLOW mask
        yellow_lower = np.array([18, 80, 80], dtype=np.uint8)
        yellow_upper = np.array([40, 255, 255], dtype=np.uint8)
        yellow_mask = cv2.inRange(hsv, yellow_lower, yellow_upper)

        detections = []
        detections.extend(self.extract_objects(red_mask, "red"))
        detections.extend(self.extract_objects(green_mask, "green"))
        detections.extend(self.extract_objects(yellow_mask, "yellow"))

        if len(detections) == 0:
            self.publish_image(annotated)
            return

        best = max(detections, key=lambda d: d["area"])

        x = best["x"]
        y = best["y"]
        w = best["w"]
        h = best["h"]
        cx = best["cx"]
        cy = best["cy"]
        cls = best["class_name"]

        det = Detection2D()
        det.class_name = cls
        det.confidence = 1.0
        det.x_min = x
        det.y_min = y
        det.x_max = x + w
        det.y_max = y + h
        det.center_x = cx
        det.center_y = cy
        self.det_pub.publish(det)

        color_map = {
            "red": (0, 0, 255),
            "green": (0, 255, 0),
            "yellow": (0, 255, 255)
        }
        draw_color = color_map.get(cls, (255, 255, 255))

        cv2.rectangle(annotated, (x, y), (x + w, y + h), draw_color, 2)
        cv2.circle(annotated, (cx, cy), 4, (255, 255, 255), -1)
        cv2.putText(
            annotated,
            cls,
            (x, max(20, y - 10)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            draw_color,
            2
        )

        self.publish_image(annotated)

    def extract_objects(self, mask, class_name):
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        results = []

        for c in contours:
            area = cv2.contourArea(c)
            if area < 150:
                continue

            x, y, w, h = cv2.boundingRect(c)

            # bins ve büyük alanları ele
            if w > 250 or h > 250:
                continue

            cx = int(x + w / 2)
            cy = int(y + h / 2)

            results.append({
                "class_name": class_name,
                "area": area,
                "x": x,
                "y": y,
                "w": w,
                "h": h,
                "cx": cx,
                "cy": cy
            })

        return results

    def publish_image(self, frame):
        try:
            img_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            self.img_pub.publish(img_msg)
        except Exception as e:
            rospy.logerr("image publish error: %s", str(e))


if __name__ == "__main__":
    SimDetector()
    rospy.spin()

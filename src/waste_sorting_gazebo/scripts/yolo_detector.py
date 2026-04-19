#!/usr/bin/env python3
import os
import sys

sys.path.append('/home/ertugrulbasoren/.local/lib/python3.8/site-packages')

import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from waste_sorting_gazebo.msg import Detection2D
from ultralytics import YOLO


class YoloDetectorNode:
    def __init__(self):
        rospy.init_node("yolo_detector", anonymous=False)

        self.bridge = CvBridge()

        self.image_topic = rospy.get_param("~image_topic", "/top_camera/image_raw")
        self.model_path = rospy.get_param(
            "~model_path",
            "/home/ertugrulbasoren/waste_sorting_ws/src/waste_sorting_gazebo/models_yolo/best.pt"
        )
        self.conf_threshold = float(rospy.get_param("~conf_threshold", 0.25))
        self.publish_annotated = bool(rospy.get_param("~publish_annotated", True))

        if not os.path.exists(self.model_path):
            rospy.logerr("YOLO model not found: %s", self.model_path)
            raise FileNotFoundError(self.model_path)

        self.model = YOLO(self.model_path)

        self.det_pub = rospy.Publisher("/waste/detection", Detection2D, queue_size=10)
        self.image_pub = rospy.Publisher("/waste/detection_image", Image, queue_size=1)

        self.sub = rospy.Subscriber(
            self.image_topic,
            Image,
            self.image_callback,
            queue_size=1,
            buff_size=2**24
        )

        rospy.loginfo("YOLO detector ready. model=%s", self.model_path)

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            rospy.logerr("cv_bridge error: %s", str(e))
            return

        try:
            results = self.model.predict(
                source=frame,
                conf=self.conf_threshold,
                verbose=False
            )
        except Exception as e:
            rospy.logerr("YOLO predict error: %s", str(e))
            return

        if not results:
            return

        result = results[0]
        boxes = result.boxes

        if boxes is None or len(boxes) == 0:
            if self.publish_annotated:
                self.publish_image(frame)
            return

        best_box = None
        best_conf = -1.0

        for box in boxes:
            conf = float(box.conf[0].item())
            if conf > best_conf:
                best_conf = conf
                best_box = box

        if best_box is None:
            if self.publish_annotated:
                self.publish_image(frame)
            return

        conf = float(best_box.conf[0].item())
        cls_id = int(best_box.cls[0].item())
        _raw_class = self.model.names[cls_id]

        class_name = "object"

        xyxy = best_box.xyxy[0].cpu().numpy().astype(int)
        x1, y1, x2, y2 = xyxy.tolist()
        cx = int((x1 + x2) / 2)
        cy = int((y1 + y2) / 2)

        det_msg = Detection2D()
        det_msg.class_name = class_name
        det_msg.confidence = conf
        det_msg.x_min = x1
        det_msg.y_min = y1
        det_msg.x_max = x2
        det_msg.y_max = y2
        det_msg.center_x = cx
        det_msg.center_y = cy
        self.det_pub.publish(det_msg)

        if self.publish_annotated:
            label = f"OBJ {conf:.2f}"
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.circle(frame, (cx, cy), 4, (0, 0, 255), -1)
            cv2.putText(
                frame,
                label,
                (x1, max(20, y1 - 10)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2
            )
            self.publish_image(frame)

    def publish_image(self, frame):
        try:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.image_pub.publish(msg)
        except Exception as e:
            rospy.logerr("annotated image publish error: %s", str(e))


if __name__ == "__main__":
    try:
        YoloDetectorNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

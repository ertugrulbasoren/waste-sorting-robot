#!/usr/bin/env python3
import rospy

from gazebo_msgs.msg import ModelStates
from waste_sorting_gazebo.msg import Detection2D


class SimDetector:
    def __init__(self):
        rospy.init_node("sim_detector")

        self.image_width = int(rospy.get_param("~image_width", 1280))
        self.image_height = int(rospy.get_param("~image_height", 720))

        self.camera_x_min = float(rospy.get_param("~camera_x_min", -0.85))
        self.camera_x_max = float(rospy.get_param("~camera_x_max", 0.85))
        self.camera_y_min = float(rospy.get_param("~camera_y_min", -0.35))
        self.camera_y_max = float(rospy.get_param("~camera_y_max", 0.35))

        self.model_alias = {
            "waste_plastic": "plastic",
            "waste_metal": "metal",
            "waste_paper": "paper"
        }

        self.det_pub = rospy.Publisher("/waste/detection", Detection2D, queue_size=10)

        self.model_sub = rospy.Subscriber(
            "/gazebo/model_states",
            ModelStates,
            self.callback,
            queue_size=1
        )

        rospy.loginfo("SIM DETECTOR READY - MODEL STATES MODE")

    def callback(self, msg):
        candidates = []

        for i, model_name in enumerate(msg.name):
            if model_name not in self.model_alias:
                continue

            pose = msg.pose[i]
            x_world = pose.position.x
            y_world = pose.position.y
            z_world = pose.position.z

            if not self.is_inside_camera_area(x_world, y_world):
                continue

            px, py = self.world_to_image(x_world, y_world)

            box_w = 90
            box_h = 90

            x_min = max(0, px - box_w // 2)
            y_min = max(0, py - box_h // 2)
            x_max = min(self.image_width - 1, px + box_w // 2)
            y_max = min(self.image_height - 1, py + box_h // 2)

            candidates.append({
                "model_name": model_name,
                "class_name": self.model_alias[model_name],
                "x_world": x_world,
                "y_world": y_world,
                "z_world": z_world,
                "x_min": x_min,
                "y_min": y_min,
                "x_max": x_max,
                "y_max": y_max,
                "center_x": px,
                "center_y": py
            })

        if len(candidates) == 0:
            return

        best = max(candidates, key=lambda item: item["x_world"])

        det = Detection2D()
        det.class_name = best["class_name"]
        det.confidence = 1.0
        det.x_min = int(best["x_min"])
        det.y_min = int(best["y_min"])
        det.x_max = int(best["x_max"])
        det.y_max = int(best["y_max"])
        det.center_x = int(best["center_x"])
        det.center_y = int(best["center_y"])

        self.det_pub.publish(det)

    def is_inside_camera_area(self, x_world, y_world):
        if x_world < self.camera_x_min:
            return False
        if x_world > self.camera_x_max:
            return False
        if y_world < self.camera_y_min:
            return False
        if y_world > self.camera_y_max:
            return False
        return True

    def world_to_image(self, x_world, y_world):
        x_ratio = (x_world - self.camera_x_min) / (self.camera_x_max - self.camera_x_min)
        y_ratio = (y_world - self.camera_y_min) / (self.camera_y_max - self.camera_y_min)

        px = int(x_ratio * self.image_width)
        py = int((1.0 - y_ratio) * self.image_height)

        px = max(0, min(self.image_width - 1, px))
        py = max(0, min(self.image_height - 1, py))

        return px, py


if __name__ == "__main__":
    SimDetector()
    rospy.spin()

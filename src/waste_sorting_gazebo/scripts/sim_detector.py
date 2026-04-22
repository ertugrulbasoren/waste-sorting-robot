#!/usr/bin/env python3
import os
import rospy
from gazebo_msgs.msg import ModelStates
from waste_sorting_gazebo.msg import Detection2D


CURRENT_MODEL_FILE = os.path.expanduser("~/waste_sorting_ws/current_waste_model.txt")


class SimDetector:
    def __init__(self):
        rospy.init_node("sim_detector")

        self.model_states_sub = rospy.Subscriber(
            "/gazebo/model_states",
            ModelStates,
            self.model_states_callback,
            queue_size=1,
        )

        self.det_pub = rospy.Publisher("/waste/detection", Detection2D, queue_size=10)

        rospy.loginfo("SIM DETECTOR READY - MODEL STATES MODE")

    def get_current_model_name(self):
        if not os.path.exists(CURRENT_MODEL_FILE):
            return None

        try:
            with open(CURRENT_MODEL_FILE, "r", encoding="utf-8") as f:
                name = f.read().strip()
                return name if name else None
        except Exception:
            return None

    def infer_class_name(self, model_name):
        model_name = (model_name or "").strip().lower()

        if model_name.startswith("waste_plastic"):
            return "plastic"
        if model_name.startswith("waste_paper"):
            return "paper"
        if model_name.startswith("waste_metal"):
            return "metal"

        # eski isimlerle de uyumlu kal
        if model_name == "waste_plastic":
            return "plastic"
        if model_name == "waste_paper":
            return "paper"
        if model_name == "waste_metal":
            return "metal"

        return None

    def world_to_fake_image(self, x, y):
        # Dünya koordinatlarını yaklaşık görüntü koordinatına çeviriyoruz.
        # Tracker immediate-pick modda olduğu için çok hassas olmasına gerek yok.
        center_x = int(640 + y * 300.0)
        center_y = int(360 - x * 220.0)
        return center_x, center_y

    def model_states_callback(self, msg):
        current_model = self.get_current_model_name()
        if not current_model:
            return

        if current_model not in msg.name:
            return

        idx = msg.name.index(current_model)
        pose = msg.pose[idx]

        class_name = self.infer_class_name(current_model)
        if not class_name:
            return

        center_x, center_y = self.world_to_fake_image(
            pose.position.x,
            pose.position.y
        )

        det = Detection2D()
        det.class_name = class_name
        det.confidence = 1.0
        det.center_x = center_x
        det.center_y = center_y
        det.x_min = center_x - 20
        det.y_min = center_y - 20
        det.x_max = center_x + 20
        det.y_max = center_y + 20

        self.det_pub.publish(det)


if __name__ == "__main__":
    SimDetector()
    rospy.spin()

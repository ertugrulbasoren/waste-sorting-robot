#!/usr/bin/env python3

import os
import rospy
from std_msgs.msg import String
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import Pose

class AutoCycleManager:
    def __init__(self):
        rospy.init_node('auto_cycle_manager')

        self.colors = ['red', 'green', 'yellow']
        self.state_file = "/home/ertugrulbasoren/waste_sorting_ws/sequence_state.txt"

        self.processing = False
        self.accept_detection = False
        self.last_detected_color = None

        rospy.wait_for_service('/gazebo/spawn_sdf_model')
        rospy.wait_for_service('/gazebo/delete_model')

        self.spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        self.delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)

        self.color_sub = rospy.Subscriber('/detected_color', String, self.color_callback)

        rospy.sleep(2.0)
        self.spawn_next()

    def get_model_info(self, color):
        base = "/home/ertugrulbasoren/waste_sorting_ws/src/waste_sorting_gazebo/models"

        if color == 'red':
            return f"{base}/trash_object_red/model.sdf", "trash_red"
        elif color == 'green':
            return f"{base}/trash_object_green/model.sdf", "trash_green"
        elif color == 'yellow':
            return f"{base}/trash_object_yellow/model.sdf", "trash_yellow"
        else:
            return None, None

    def delete_all_known_models(self):
        for name in ['trash_red', 'trash_green', 'trash_yellow']:
            try:
                self.delete_model(name)
                rospy.loginfo("Deleted model: %s", name)
            except:
                pass

    def load_index(self):
        if not os.path.exists(self.state_file):
            return 0

        try:
            with open(self.state_file, 'r') as f:
                value = int(f.read().strip())
                return value % len(self.colors)
        except:
            return 0

    def save_index(self, index):
        with open(self.state_file, 'w') as f:
            f.write(str(index))

    def enable_detection(self, event):
        self.accept_detection = True
        rospy.loginfo("Detection acceptance enabled")

    def spawn_next(self):
        index = self.load_index()
        color = self.colors[index]
        model_path, model_name = self.get_model_info(color)

        with open(model_path, 'r') as f:
            model_xml = f.read()

        pose = Pose()
        pose.position.x = -0.8
        pose.position.y = 0.0
        pose.position.z = 0.2

        try:
            self.accept_detection = False
            self.last_detected_color = None

            self.spawn_model(model_name, model_xml, "", pose, "world")
            rospy.loginfo("Spawned next object: %s", color)

            next_index = (index + 1) % len(self.colors)
            self.save_index(next_index)

            self.processing = False

            rospy.Timer(rospy.Duration(2.0), self.enable_detection, oneshot=True)

        except rospy.ServiceException as e:
            rospy.logerr("Spawn failed: %s", str(e))

    def color_callback(self, msg):
        if self.processing:
            return

        if not self.accept_detection:
            return

        detected_color = msg.data

        if detected_color not in self.colors:
            return

        self.processing = True
        self.accept_detection = False
        self.last_detected_color = detected_color

        rospy.loginfo("Cycle manager accepted color: %s", detected_color)
        rospy.Timer(rospy.Duration(5.0), self.finish_cycle, oneshot=True)

    def finish_cycle(self, event):
        rospy.loginfo("Finishing cycle for color: %s", self.last_detected_color)
        self.delete_all_known_models()
        rospy.sleep(1.0)
        self.spawn_next()

if __name__ == '__main__':
    AutoCycleManager()
    rospy.spin()

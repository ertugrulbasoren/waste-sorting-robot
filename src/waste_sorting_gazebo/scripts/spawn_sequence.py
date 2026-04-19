#!/usr/bin/env python3

import rospy
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import Pose

class SpawnSequence:
    def __init__(self):
        rospy.init_node('spawn_sequence')

        self.colors = ['red', 'green', 'yellow']
        self.index = 0

        rospy.wait_for_service('/gazebo/spawn_sdf_model')
        rospy.wait_for_service('/gazebo/delete_model')

        self.spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        self.delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)

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
                rospy.loginfo("Deleted existing model: %s", name)
            except:
                pass

    def spawn_next(self):
        color = self.colors[self.index]
        model_path, model_name = self.get_model_info(color)

        with open(model_path, 'r') as f:
            model_xml = f.read()

        pose = Pose()
        pose.position.x = -0.8
        pose.position.y = 0.0
        pose.position.z = 0.2

        self.spawn_model(model_name, model_xml, "", pose, "world")
        rospy.loginfo("Spawned next object: %s", color)

        self.index = (self.index + 1) % len(self.colors)

if __name__ == '__main__':
    spawner = SpawnSequence()
    spawner.delete_all_known_models()
    spawner.spawn_next()

#!/usr/bin/env python3

import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose

def spawn():
    rospy.init_node('spawn_trash')

    color = rospy.get_param('~color', 'red')

    if color == 'red':
        model_path = "/home/ertugrulbasoren/waste_sorting_ws/src/waste_sorting_gazebo/models/trash_object_red/model.sdf"
        model_name = "trash_red"
        spawn_y = -0.15
    elif color == 'green':
        model_path = "/home/ertugrulbasoren/waste_sorting_ws/src/waste_sorting_gazebo/models/trash_object_green/model.sdf"
        model_name = "trash_green"
        spawn_y = 0.0
    elif color == 'yellow':
        model_path = "/home/ertugrulbasoren/waste_sorting_ws/src/waste_sorting_gazebo/models/trash_object_yellow/model.sdf"
        model_name = "trash_yellow"
        spawn_y = 0.15
    else:
        rospy.logerr("Unknown color: %s", color)
        return

    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)

    with open(model_path, "r") as f:
        model_xml = f.read()

    pose = Pose()
    pose.position.x = -0.8
    pose.position.y = spawn_y
    pose.position.z = 0.2

    try:
        spawn_model(model_name, model_xml, "", pose, "world")
        rospy.loginfo("Spawned %s as %s", color, model_name)
    except rospy.ServiceException as e:
        rospy.logerr("Spawn failed: %s", str(e))

if __name__ == "__main__":
    spawn()

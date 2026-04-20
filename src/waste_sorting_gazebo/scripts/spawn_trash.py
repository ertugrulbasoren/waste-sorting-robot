#!/usr/bin/env python3
import os
import rospy
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import Pose

MODEL_BASE_PATH = os.path.expanduser("~/waste_sorting_ws/src/waste_sorting_gazebo/models")

MODEL_INFO = {
    "plastic": {
        "model_name": "waste_plastic",
        "model_dir": "waste_plastic"
    },
    "metal": {
        "model_name": "waste_metal",
        "model_dir": "waste_metal"
    },
    "paper": {
        "model_name": "waste_paper",
        "model_dir": "waste_paper"
    }
}
def load_model_sdf(model_dir_name):
    model_path = os.path.join(MODEL_BASE_PATH, model_dir_name, "model.sdf")
    with open(model_path, "r") as f:
        return f.read()

if __name__ == "__main__":
    rospy.init_node("spawn_trash")

    waste_type = rospy.get_param("~waste_type", "plastic")

    if waste_type not in MODEL_INFO:
        rospy.logerr("Unknown waste_type: %s", waste_type)
        raise SystemExit(1)

    model_name = MODEL_INFO[waste_type]["model_name"]
    model_dir = MODEL_INFO[waste_type]["model_dir"]

    rospy.wait_for_service("/gazebo/delete_model")
    rospy.wait_for_service("/gazebo/spawn_sdf_model")

    delete_model = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
    spawn_model = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)

    try:
        delete_model(model_name)
    except Exception:
        pass

    model_xml = load_model_sdf(model_dir)

    pose = Pose()
    pose.position.x = -0.6
    pose.position.y = 0.0
    pose.position.z = 0.25
    pose.orientation.x = 0.0
    pose.orientation.y = 0.0
    pose.orientation.z = 0.0
    pose.orientation.w = 1.0

    spawn_model(model_name, model_xml, "", pose, "world")
    rospy.loginfo("Spawned %s as %s", waste_type, model_name)

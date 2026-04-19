#!/usr/bin/env python3
import rospy
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import Pose

BOX_SDF_TEMPLATE = """
<sdf version='1.6'>
  <model name='{name}'>
    <static>false</static>
    <link name='link'>
      <inertial>
        <mass>0.1</mass>
        <inertia>
          <ixx>0.0001</ixx>
          <iyy>0.0001</iyy>
          <izz>0.0001</izz>
          <ixy>0.0</ixy>
          <ixz>0.0</ixz>
          <iyz>0.0</iyz>
        </inertia>
      </inertial>
      <collision name='collision'>
        <geometry>
          <box>
            <size>0.06 0.06 0.06</size>
          </box>
        </geometry>
        <surface>
          <friction>
            <ode>
              <mu>10.0</mu>
              <mu2>10.0</mu2>
            </ode>
          </friction>
        </surface>
      </collision>
      <visual name='visual'>
        <geometry>
          <box>
            <size>0.06 0.06 0.06</size>
          </box>
        </geometry>
        <material>
          <ambient>{r} {g} {b} 1</ambient>
          <diffuse>{r} {g} {b} 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""

def get_color(color_name):
    if color_name == "red":
        return (1, 0, 0)
    if color_name == "green":
        return (0, 1, 0)
    if color_name == "yellow":
        return (1, 1, 0)
    return (0, 1, 0)

if __name__ == "__main__":
    rospy.init_node("spawn_trash")

    color = rospy.get_param("~color", "green")
    model_name = f"trash_{color}"

    rospy.wait_for_service("/gazebo/delete_model")
    rospy.wait_for_service("/gazebo/spawn_sdf_model")

    delete_model = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
    spawn_model = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)

    try:
        delete_model(model_name)
    except Exception:
        pass

    r, g, b = get_color(color)
    model_xml = BOX_SDF_TEMPLATE.format(name=model_name, r=r, g=g, b=b)

    pose = Pose()
    pose.position.x = -0.6
    pose.position.y = 0.0
    pose.position.z = 0.25
    pose.orientation.w = 1.0

    spawn_model(model_name, model_xml, "", pose, "world")
    rospy.loginfo("Spawned %s as %s", color, model_name)

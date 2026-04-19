#!/usr/bin/env python3
import rospy
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import Pose

ARM_BASE_SDF = """
<sdf version='1.6'>
  <model name='simple_sort_arm'>
    <static>true</static>

    <link name='base_link'>
      <pose>0 0 0.20 0 0 0</pose>

      <collision name='base_collision'>
        <geometry>
          <cylinder>
            <radius>0.08</radius>
            <length>0.40</length>
          </cylinder>
        </geometry>
      </collision>

      <visual name='base_visual'>
        <geometry>
          <cylinder>
            <radius>0.08</radius>
            <length>0.40</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>0.15 0.15 0.15 1</ambient>
          <diffuse>0.15 0.15 0.15 1</diffuse>
        </material>
      </visual>
    </link>

    <link name='arm_link_1'>
      <pose>0.18 0 0.42 0 0 0</pose>

      <collision name='arm1_collision'>
        <geometry>
          <box>
            <size>0.36 0.05 0.05</size>
          </box>
        </geometry>
      </collision>

      <visual name='arm1_visual'>
        <geometry>
          <box>
            <size>0.36 0.05 0.05</size>
          </box>
        </geometry>
        <material>
          <ambient>0.10 0.40 0.85 1</ambient>
          <diffuse>0.10 0.40 0.85 1</diffuse>
        </material>
      </visual>
    </link>

    <link name='arm_link_2'>
      <pose>0.42 0 0.32 0 0 -0.55</pose>

      <collision name='arm2_collision'>
        <geometry>
          <box>
            <size>0.30 0.045 0.045</size>
          </box>
        </geometry>
      </collision>

      <visual name='arm2_visual'>
        <geometry>
          <box>
            <size>0.30 0.045 0.045</size>
          </box>
        </geometry>
        <material>
          <ambient>0.10 0.75 0.30 1</ambient>
          <diffuse>0.10 0.75 0.30 1</diffuse>
        </material>
      </visual>
    </link>

    <link name='arm_link_3'>
      <pose>0.57 0 0.22 0 0 -0.15</pose>

      <collision name='arm3_collision'>
        <geometry>
          <box>
            <size>0.16 0.04 0.04</size>
          </box>
        </geometry>
      </collision>

      <visual name='arm3_visual'>
        <geometry>
          <box>
            <size>0.16 0.04 0.04</size>
          </box>
        </geometry>
        <material>
          <ambient>0.95 0.82 0.10 1</ambient>
          <diffuse>0.95 0.82 0.10 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""

GRIPPER_SDF = """
<sdf version='1.6'>
  <model name='simple_sort_gripper'>
    <static>false</static>

    <link name='gripper_link'>
      <inertial>
        <mass>0.2</mass>
        <inertia>
          <ixx>0.001</ixx>
          <iyy>0.001</iyy>
          <izz>0.001</izz>
          <ixy>0.0</ixy>
          <ixz>0.0</ixz>
          <iyz>0.0</iyz>
        </inertia>
      </inertial>

      <collision name='collision'>
        <geometry>
          <box>
            <size>0.10 0.05 0.03</size>
          </box>
        </geometry>
      </collision>

      <visual name='visual'>
        <geometry>
          <box>
            <size>0.10 0.05 0.03</size>
          </box>
        </geometry>
        <material>
          <ambient>0.95 0.20 0.20 1</ambient>
          <diffuse>0.95 0.20 0.20 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""

if __name__ == "__main__":
    rospy.init_node("spawn_simple_arm")

    rospy.wait_for_service("/gazebo/delete_model")
    rospy.wait_for_service("/gazebo/spawn_sdf_model")

    delete_model = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
    spawn_model = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)

    for name in ["simple_sort_arm", "simple_sort_gripper"]:
        try:
            delete_model(name)
        except Exception:
            pass

    base_pose = Pose()
    base_pose.position.x = 0.95
    base_pose.position.y = -0.75
    base_pose.position.z = 0.0
    base_pose.orientation.w = 1.0

    gripper_pose = Pose()
    gripper_pose.position.x = 0.62
    gripper_pose.position.y = 0.00
    gripper_pose.position.z = 0.55
    gripper_pose.orientation.w = 1.0

    spawn_model("simple_sort_arm", ARM_BASE_SDF, "", base_pose, "world")
    spawn_model("simple_sort_gripper", GRIPPER_SDF, "", gripper_pose, "world")

    rospy.loginfo("Spawned simple_sort_arm and simple_sort_gripper")

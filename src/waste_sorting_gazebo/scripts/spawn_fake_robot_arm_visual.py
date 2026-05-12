#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Spawn a simple visual-only robot arm model in Gazebo.

Purpose:
  This is the first safe robot-arm integration step.
  It does not control waste objects yet.
  It only places a simple industrial-looking arm near the conveyor so that
  the final demo scene visually includes a robot arm.

Why visual-only first:
  The current realistic YOLO sorting pipeline is stable.
  We do not want to break it with UR5/MoveIt complexity immediately.
  This fake arm will later be replaced or extended with pick-place animation.

Usage:
  rosrun waste_sorting_gazebo spawn_fake_robot_arm_visual.py

Optional params:
  _model_name:=fake_sorting_arm
  _x:=0.55
  _y:=0.85
  _z:=0.00
  _yaw:=-1.5708
"""

import math
import rospy
from gazebo_msgs.srv import DeleteModel, SpawnModel
from geometry_msgs.msg import Pose, Point, Quaternion


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def build_fake_arm_sdf(model_name: str) -> str:
    return """<?xml version="1.0"?>
<sdf version="1.6">
  <model name="{model_name}">
    <static>true</static>

    <!--
      Visual-only simplified industrial robot arm.

      Coordinate convention:
        The model origin is at the base center.
        The whole model can be positioned with spawn pose.
    -->

    <link name="base_link">
      <pose>0 0 0.06 0 0 0</pose>

      <visual name="base_visual">
        <geometry>
          <cylinder>
            <radius>0.16</radius>
            <length>0.12</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>0.08 0.08 0.08 1</ambient>
          <diffuse>0.12 0.12 0.12 1</diffuse>
          <specular>0.2 0.2 0.2 1</specular>
        </material>
      </visual>

      <collision name="base_collision">
        <geometry>
          <cylinder>
            <radius>0.16</radius>
            <length>0.12</length>
          </cylinder>
        </geometry>
      </collision>
    </link>

    <link name="shoulder_link">
      <pose>0 0 0.32 0.0 0.45 0</pose>

      <visual name="shoulder_visual">
        <geometry>
          <box>
            <size>0.16 0.16 0.48</size>
          </box>
        </geometry>
        <material>
          <ambient>0.85 0.85 0.85 1</ambient>
          <diffuse>0.95 0.95 0.95 1</diffuse>
          <specular>0.2 0.2 0.2 1</specular>
        </material>
      </visual>

      <collision name="shoulder_collision">
        <geometry>
          <box>
            <size>0.16 0.16 0.48</size>
          </box>
        </geometry>
      </collision>
    </link>

    <link name="upper_arm_link">
      <pose>-0.22 0 0.62 0 1.15 0</pose>

      <visual name="upper_arm_visual">
        <geometry>
          <box>
            <size>0.55 0.12 0.12</size>
          </box>
        </geometry>
        <material>
          <ambient>0.95 0.95 0.95 1</ambient>
          <diffuse>1.0 1.0 1.0 1</diffuse>
          <specular>0.2 0.2 0.2 1</specular>
        </material>
      </visual>

      <collision name="upper_arm_collision">
        <geometry>
          <box>
            <size>0.55 0.12 0.12</size>
          </box>
        </geometry>
      </collision>
    </link>

    <link name="forearm_link">
      <pose>-0.48 0 0.46 0 0.35 0</pose>

      <visual name="forearm_visual">
        <geometry>
          <box>
            <size>0.48 0.10 0.10</size>
          </box>
        </geometry>
        <material>
          <ambient>0.95 0.95 0.95 1</ambient>
          <diffuse>1.0 1.0 1.0 1</diffuse>
          <specular>0.2 0.2 0.2 1</specular>
        </material>
      </visual>

      <collision name="forearm_collision">
        <geometry>
          <box>
            <size>0.48 0.10 0.10</size>
          </box>
        </geometry>
      </collision>
    </link>

    <link name="wrist_link">
      <pose>-0.72 0 0.38 0 0 0</pose>

      <visual name="wrist_visual">
        <geometry>
          <sphere>
            <radius>0.08</radius>
          </sphere>
        </geometry>
        <material>
          <ambient>0.15 0.15 0.15 1</ambient>
          <diffuse>0.20 0.20 0.20 1</diffuse>
          <specular>0.3 0.3 0.3 1</specular>
        </material>
      </visual>

      <collision name="wrist_collision">
        <geometry>
          <sphere>
            <radius>0.08</radius>
          </sphere>
        </geometry>
      </collision>
    </link>

    <link name="gripper_left_link">
      <pose>-0.82 0.055 0.35 0 0 0</pose>

      <visual name="gripper_left_visual">
        <geometry>
          <box>
            <size>0.16 0.025 0.035</size>
          </box>
        </geometry>
        <material>
          <ambient>0.05 0.05 0.05 1</ambient>
          <diffuse>0.05 0.05 0.05 1</diffuse>
          <specular>0.2 0.2 0.2 1</specular>
        </material>
      </visual>

      <collision name="gripper_left_collision">
        <geometry>
          <box>
            <size>0.16 0.025 0.035</size>
          </box>
        </geometry>
      </collision>
    </link>

    <link name="gripper_right_link">
      <pose>-0.82 -0.055 0.35 0 0 0</pose>

      <visual name="gripper_right_visual">
        <geometry>
          <box>
            <size>0.16 0.025 0.035</size>
          </box>
        </geometry>
        <material>
          <ambient>0.05 0.05 0.05 1</ambient>
          <diffuse>0.05 0.05 0.05 1</diffuse>
          <specular>0.2 0.2 0.2 1</specular>
        </material>
      </visual>

      <collision name="gripper_right_collision">
        <geometry>
          <box>
            <size>0.16 0.025 0.035</size>
          </box>
        </geometry>
      </collision>
    </link>

  </model>
</sdf>
""".format(model_name=model_name)


def main():
    rospy.init_node("spawn_fake_robot_arm_visual", anonymous=False)

    model_name = rospy.get_param("~model_name", "fake_sorting_arm")

    x = float(rospy.get_param("~x", 0.55))
    y = float(rospy.get_param("~y", 0.85))
    z = float(rospy.get_param("~z", 0.00))
    yaw = float(rospy.get_param("~yaw", -1.5708))

    delete_existing = bool(rospy.get_param("~delete_existing", True))

    rospy.loginfo("[FAKE ARM VISUAL] Waiting for Gazebo services...")
    rospy.wait_for_service("/gazebo/spawn_sdf_model")
    rospy.wait_for_service("/gazebo/delete_model")

    spawn_srv = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
    delete_srv = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)

    if delete_existing:
        try:
            delete_srv(model_name)
        except Exception:
            pass

    pose = Pose()
    pose.position = Point(x, y, z)
    pose.orientation = yaw_to_quaternion(yaw)

    sdf_xml = build_fake_arm_sdf(model_name)

    response = spawn_srv(
        model_name,
        sdf_xml,
        "",
        pose,
        "world",
    )

    if response.success:
        rospy.loginfo(
            "[FAKE ARM VISUAL] Spawned model=%s pose=(%.3f, %.3f, %.3f) yaw=%.3f",
            model_name,
            x,
            y,
            z,
            yaw,
        )
    else:
        rospy.logerr(
            "[FAKE ARM VISUAL] Spawn failed model=%s status=%s",
            model_name,
            response.status_message,
        )


if __name__ == "__main__":
    main()

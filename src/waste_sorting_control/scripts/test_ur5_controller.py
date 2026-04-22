#!/usr/bin/env python3
import rospy
import actionlib

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint


def send_goal(client, positions, duration):
    goal = FollowJointTrajectoryGoal()
    goal.trajectory.joint_names = [
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint",
    ]

    point = JointTrajectoryPoint()
    point.positions = positions
    point.velocities = [0.0] * 6
    point.time_from_start = rospy.Duration(duration)

    goal.trajectory.points.append(point)
    goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(0.5)

    client.send_goal(goal)
    client.wait_for_result(rospy.Duration(duration + 5.0))

    result = client.get_result()
    state = client.get_state()
    rospy.loginfo("Finished. state=%s result=%s", str(state), str(result))


def main():
    rospy.init_node("test_ur5_controller")

    client = actionlib.SimpleActionClient(
        "/pos_joint_traj_controller/follow_joint_trajectory",
        FollowJointTrajectoryAction
    )

    rospy.loginfo("Waiting for controller action server...")
    client.wait_for_server()
    rospy.loginfo("Connected.")

    pose_a = [0.0, -1.20, 1.20, -1.57, -1.57, 0.0]
    pose_b = [1.20, -1.80, 1.80, -1.20, -1.57, 0.80]

    rospy.loginfo("Sending pose A")
    send_goal(client, pose_a, 3.0)

    rospy.sleep(1.0)

    rospy.loginfo("Sending pose B")
    send_goal(client, pose_b, 4.0)


if __name__ == "__main__":
    main()

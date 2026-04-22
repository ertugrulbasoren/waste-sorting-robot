#!/usr/bin/env python3
import sys
import rospy
import moveit_commander


def main():
    rospy.init_node("ur5_go_home", anonymous=True)

    moveit_commander.roscpp_initialize(sys.argv)

    group_name = "manipulator"
    group = moveit_commander.MoveGroupCommander(group_name)

    group.set_planning_time(10.0)
    group.set_num_planning_attempts(10)
    group.set_max_velocity_scaling_factor(0.2)
    group.set_max_acceleration_scaling_factor(0.2)

    rospy.loginfo("Connected to MoveIt group: %s", group_name)

    home_joints = [
        0.0,
        -1.57,
        1.57,
        -1.57,
        -1.57,
        0.0
    ]

    rospy.loginfo("Sending UR5 to home joint pose...")
    success = group.go(home_joints, wait=True)
    group.stop()
    group.clear_pose_targets()

    if success:
        rospy.loginfo("UR5 reached home pose")
    else:
        rospy.logerr("UR5 failed to reach home pose")

    moveit_commander.roscpp_shutdown()


if __name__ == "__main__":
    main()

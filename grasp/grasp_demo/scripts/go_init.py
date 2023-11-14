#! /usr/bin/env python
import sys
import rospy
import moveit_commander
import geometry_msgs
import tf


moveit_commander.roscpp_initializer.roscpp_initialize(sys.argv)
rospy.init_node('go_init', anonymous=True)
# robot = moveit_commander.robot.RobotCommander()

arm_group = moveit_commander.move_group.MoveGroupCommander("arm_with_torso")
gripper_group = moveit_commander.move_group.MoveGroupCommander("gripper")
joint_state_positions = gripper_group.get_current_joint_values()
print(str(joint_state_positions))

arm_group.set_joint_value_target([-9.487505139319357e-07, -1.3286165817724758, 0.9411910978981766, -2.632872053084035, -2.217766946445675, -1.2135210418771862, 1.477427293130634, 1.4762755298156103])
arm_group.go(wait=True)

# kk = gripper_group.get_joint_value_target()
# print(kk)

moveit_commander.roscpp_initializer.roscpp_shutdown()

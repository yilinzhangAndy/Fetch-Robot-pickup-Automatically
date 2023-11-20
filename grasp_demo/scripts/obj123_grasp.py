#! /usr/bin/env python
#-*- coding: utf-8 -*-

import tf
import sys
import rospy
from control_msgs.msg import GripperCommandActionGoal
from geometry_msgs.msg import PoseStamped, Pose
from darknet_ros_msgs.msg import BoundingBoxes
from message_filters import ApproximateTimeSynchronizer, Subscriber
from moveit_python import MoveGroupInterface, PlanningSceneInterface

 

class MoveRobot():
    def __init__(self):

 

        # 初始化 planning group
        self.move_group = MoveGroupInterface("arm_with_torso", "base_link")
        self.gripper_group = MoveGroupInterface("gripper", "gripper_link")

 
        self.arm_group.set_max_acceleration_scaling_factor(1)
        self.arm_group.set_max_velocity_scaling_factor(1)


        # 物体的位置
        self.Obj_pose = PoseStamped()
        self.Obj_pose.pose.position.x = 0
        self.find_enable = False
        self.Obj_class = ''

 

        # Subscriber initialization remains the same
        self.obj_pose_sub = Subscriber("/objection_position_pose", Pose)
        self.yolo_sub = Subscriber("/darknet_ros/BoundingBoxes", BoundingBoxes)

 

        # Sync Subscribers
        self.ats = ApproximateTimeSynchronizer(
            [
                self.obj_pose_sub,
                self.yolo_sub
            ],
            queue_size=5,
            slop=1,
            allow_headerless=True
        )
        self.ats.registerCallback(self.msg_filter_callback)

 

    def msg_filter_callback(self, msg, yolo_msg):
        if self.find_enable:
            self.Obj_pose.pose = msg
            self.Obj_class = yolo_msg.bounding_boxes[0].Class
        if self.Obj_pose.pose.position.x != 0:
            self.find_enable = False

 

    def ObjectCallback(self, msg):
        if self.find_enable:
            self.Obj_pose.pose = msg
        if self.Obj_pose.pose.position.x != 0:
            self.find_enable = False

 

    def stop(self):
        # Close connections and stop the robot. Actual implementation might differ depending on your requirements.
        pass

 

    def gripper_move(self, width):
        # Assuming you have a way to set the joint values of the gripper and move it.
        # Exact implementation might depend on your setup and requirements.
        pass

 

    def gazeboAttach(self):
        rospy.loginfo("Attaching gripper and object")
        req = AttachRequest()
        req.model_name_1 = "fetch"
        req.link_name_1 = "wrist_roll_link"
        req.model_name_2 = "coke_can"
        req.link_name_2 = "link"
        self.attach_srv.call(req)

 

    def gazeboDetach(self):
        rospy.loginfo("Detaching gripper and object")
        req = AttachRequest()
        req.model_name_1 = "fetch"
        req.link_name_1 = "wrist_roll_link"
        req.model_name_2 = "coke_can"
        req.link_name_2 = "link"

 

    def plan_cartesian_path(self, pose):
        # Exact implementation might differ depending on your requirements. 
        # The MoveGroupInterface does not provide a direct compute_cartesian_path equivalent. 
        # You might need to compute waypoints and then use those for planning.

 

    def goSP(self):
        joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
                       "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
        joint_values = [-9.487505139319357e-07, -1.3286165817724758, 0.9411910978981766, -
                        2.632872053084035, -2.217766946445675, -1.2135210418771862, 1.477427293130634, 1.4762755298156103]
        self.move_group.moveToJointPosition(joint_names, joint_values)

 

    def grasp_obj(self):
        # Exact implementation might differ depending on your requirements. 
        pass

 

    def move_obj(self):
        # Exact implementation might differ depending on your requirements.
        pass

 

    def main_loop(self):
        try:
            self.gripper_move(0.050)
            self.find_enable = True
            rospy.sleep(3)  # 2. 识别当前的抓取位姿态(3s)
            if self.find_enable == False:
                self.goSP()  # 1. 去到预抓取位置
                self.grasp_obj()
                rospy.sleep(1)
                self.gripper_move(0.036)
                self.move_obj()
                self.Obj_pose.pose.position.x = 0
            else:
                rospy.logwarn('cant find object')
        except Exception as e:#! /usr/bin/env python
#-*- coding: utf-8 -*-

 

import tf
import sys
import rospy
from control_msgs.msg import GripperCommandActionGoal
from geometry_msgs.msg import PoseStamped, Pose
from darknet_ros_msgs.msg import BoundingBoxes
from message_filters import ApproximateTimeSynchronizer, Subscriber
from moveit_python import MoveGroupInterface, PlanningSceneInterface

 

class MoveRobot():
    def __init__(self):

 

        # 初始化 planning group
        self.move_group = MoveGroupInterface("arm_with_torso", "base_link")
        self.gripper_group = MoveGroupInterface("gripper", "gripper_link")

 

        # 物体的位置
        self.Obj_pose = PoseStamped()
        self.Obj_pose.pose.position.x = 0
        self.find_enable = False
        self.Obj_class = ''

 

        # Subscriber initialization remains the same
        self.obj_pose_sub = Subscriber("/objection_position_pose", Pose)
        self.yolo_sub = Subscriber("/darknet_ros/BoundingBoxes", BoundingBoxes)

 

        # Sync Subscribers
        self.ats = ApproximateTimeSynchronizer(
            [
                self.obj_pose_sub,
                self.yolo_sub
            ],
            queue_size=5,
            slop=1,
            allow_headerless=True
        )
        self.ats.registerCallback(self.msg_filter_callback)

 

    def msg_filter_callback(self, msg, yolo_msg):
        if self.find_enable:
            self.Obj_pose.pose = msg
            self.Obj_class = yolo_msg.bounding_boxes[0].Class
        if self.Obj_pose.pose.position.x != 0:
            self.find_enable = False

 

    def ObjectCallback(self, msg):
        if self.find_enable:
            self.Obj_pose.pose = msg
        if self.Obj_pose.pose.position.x != 0:
            self.find_enable = False

 

    def stop(self):
        # Close connections and stop the robot. Actual implementation might differ depending on your requirements.
        pass

 

    def gripper_move(self, width):
        # Assuming you have a way to set the joint values of the gripper and move it.
        # Exact implementation might depend on your setup and requirements.
        pass

 

    def gazeboAttach(self):
        rospy.loginfo("Attaching gripper and object")
        req = AttachRequest()
        req.model_name_1 = "fetch"
        req.link_name_1 = "wrist_roll_link"
        req.model_name_2 = "coke_can"
        req.link_name_2 = "link"
        self.attach_srv.call(req)

 

    def gazeboDetach(self):
        rospy.loginfo("Detaching gripper and object")
        req = AttachRequest()
        req.model_name_1 = "fetch"
        req.link_name_1 = "wrist_roll_link"
        req.model_name_2 = "coke_can"
        req.link_name_2 = "link"

 

    def plan_cartesian_path(self, pose):
        # Exact implementation might differ depending on your requirements. 
        # The MoveGroupInterface does not provide a direct compute_cartesian_path equivalent. 
        # You might need to compute waypoints and then use those for planning.

 

    def goSP(self):
        joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
                       "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
        joint_values = [-9.487505139319357e-07, -1.3286165817724758, 0.9411910978981766, -
                        2.632872053084035, -2.217766946445675, -1.2135210418771862, 1.477427293130634, 1.4762755298156103]
        self.move_group.moveToJointPosition(joint_names, joint_values)

 

    def grasp_obj(self):
        # Exact implementation might differ depending on your requirements. 
        pass

 

    def move_obj(self):
        # Exact implementation might differ depending on your requirements.
        pass

 

    def main_loop(self):
        try:
            self.gripper_move(0.050)
            self.find_enable = True
            rospy.sleep(3)  # 2. 识别当前的抓取位姿态(3s)
            if self.find_enable == False:
                self.goSP()  # 1. 去到预抓取位置
                self.grasp_obj()
                rospy.sleep(1)
                self.gripper_move(0.036)
                self.move_obj()
                self.Obj_pose.pose.position.x = 0
            else:
                rospy.logwarn('cant find object')
        except Exception as e:[2:47 PM] Zhang, Yilin

#! /usr/bin/env python

#-*- coding: utf-8 -*-

 

import tf

import sys

import rospy

from control_msgs.msg import GripperCommandActionGoal

from geometry_msgs.msg import PoseStamped, Pose

from darknet_ros_msgs.msg import BoundingBoxes

from message_filters import ApproximateTimeSynchronizer, Subscriber

from moveit_python import MoveGroupInterface, PlanningSceneInterface

 

class MoveRobot():

    def __init__(self):

 

        # 初始化 planning group

        self.move_group = MoveGroupInterface("arm_with_torso", "base_link")

        self.gripper_group = MoveGroupInterface("gripper", "gripper_link")

 

        # 物体的位置

        self.Obj_pose = PoseStamped()

        self.Obj_pose.pose.position.x = 0

        self.find_enable = False

        self.Obj_class = ''

 

        # Subscriber initialization remains the same

        self.obj_pose_sub = Subscriber("/objection_position_pose", Pose)

        self.yolo_sub = Subscriber("/darknet_ros/BoundingBoxes", BoundingBoxes)

 

        # Sync Subscribers

        self.ats = ApproximateTimeSynchronizer(

            [

                self.obj_pose_sub,

                self.yolo_sub

            ],

            queue_size=5,

            slop=1,

            allow_headerless=True

        )

        self.ats.registerCallback(self.msg_filter_callback)

 

    def msg_filter_callback(self, msg, yolo_msg):

        if self.find_enable:

            self.Obj_pose.pose = msg

            self.Obj_class = yolo_msg.bounding_boxes[0].Class

        if self.Obj_pose.pose.position.x != 0:

            self.find_enable = False

 

    def ObjectCallback(self, msg):

        if self.find_enable:

            self.Obj_pose.pose = msg

        if self.Obj_pose.pose.position.x != 0:

            self.find_enable = False

 

    def stop(self):

        # Close connections and stop the robot. Actual implementation might differ depending on your requirements.

        pass

 

    def gripper_move(self, width):

        # Assuming you have a way to set the joint values of the gripper and move it.

        # Exact implementation might depend on your setup and requirements.

        pass

 

    def gazeboAttach(self):

        rospy.loginfo("Attaching gripper and object")

        req = AttachRequest()

        req.model_name_1 = "fetch"

        req.link_name_1 = "wrist_roll_link"

        req.model_name_2 = "coke_can"

        req.link_name_2 = "link"

        self.attach_srv.call(req)

 

    def gazeboDetach(self):

        rospy.loginfo("Detaching gripper and object")

        req = AttachRequest()

        req.model_name_1 = "fetch"

        req.link_name_1 = "wrist_roll_link"

        req.model_name_2 = "coke_can"

        req.link_name_2 = "link"

 

    def plan_cartesian_path(self, pose):

        # Exact implementation might differ depending on your requirements. 

        # The MoveGroupInterface does not provide a direct compute_cartesian_path equivalent. 

        # You might need to compute waypoints and then use those for planning.

 

    def goSP(self):

        joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",

                       "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]

        joint_values = [-9.487505139319357e-07, -1.3286165817724758, 0.9411910978981766, -

                        2.632872053084035, -2.217766946445675, -1.2135210418771862, 1.477427293130634, 1.4762755298156103]

        self.move_group.moveToJointPosition(joint_names, joint_values)

 

    def grasp_obj(self):

        # Exact implementation might differ depending on your requirements. 

        pass

 

    def move_obj(self):

        # Exact implementation might differ depending on your requirements.

        pass

 

    def main_loop(self):

        try:

            self.gripper_move(0.050)

            self.find_enable = True

            rospy.sleep(3)  # 2. 识别当前的抓取位姿态(3s)

            if self.find_enable == False:

                self.goSP()  # 1. 去到预抓取位置

                self.grasp_obj()

                rospy.sleep(1)

                self.gripper_move(0.036)

                self.move_obj()

                self.Obj_pose.pose.position.x = 0

            else:

                rospy.logwarn('cant find object')

        except Exception as e:


#! /usr/bin/env python
#-*- coding: utf-8 -*-
import tf
import sys
import rospy
import moveit_commander
from control_msgs.msg import GripperCommandActionGoal
from geometry_msgs.msg import PoseStamped, Pose
from darknet_ros_msgs.msg import BoundingBoxes
from message_filters import ApproximateTimeSynchronizer, Subscriber



class MoveRobot():
    def __init__(self):

        self.robot = moveit_commander.robot.RobotCommander()
        self.arm_group = moveit_commander.move_group.MoveGroupCommander(
            "arm_with_torso")

        self.gripper_group = moveit_commander.move_group.MoveGroupCommander(
            "gripper")

        self.arm_group.set_max_acceleration_scaling_factor(1)
        self.arm_group.set_max_velocity_scaling_factor(1)

        
        self.Obj_pose = PoseStamped()
        self.Obj_pose.pose.position.x = 0
        self.find_enable = False
        self.Obj_class = ''

       
        self.obj_pose_sub = Subscriber(
            "/objection_position_pose", Pose)

        self.yolo_sub = Subscriber("/darknet_ros/BoundingBoxes", BoundingBoxes)

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
        moveit_commander.roscpp_initializer.roscpp_shutdown()

    def gripper_move(self, width):
        gripper_joint_values = self.gripper_group.get_current_joint_values()
        gripper_joint_values[0] = width
        gripper_joint_values[1] = width


        self.gripper_group.set_joint_value_target(gripper_joint_values)
        self.gripper_group.go()

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
        """
           

            Parameters:
                pose - pose

            Returns:
                None
            """
        waypoints = []
        waypoints.append(pose)

        
        self.arm_group.set_start_state_to_current_state()

     
        (plan, fraction) = self.arm_group.compute_cartesian_path(
            waypoints,   # waypoint poses，
            0.01,        # eef_step，
            False)        # avoid_collisions，
        self.arm_group.execute(plan, wait=True)

    def goSP(self):

        self.arm_group.set_joint_value_target([-9.487505139319357e-07, -1.3286165817724758, 0.9411910978981766, -
                                              2.632872053084035, -2.217766946445675, -1.2135210418771862, 1.477427293130634, 1.4762755298156103])

        self.arm_group.go(wait=True)
        kj = self.arm_group.get_current_joint_values()
        print(kj)

    def grasp_obj(self):

      
        print(self.Obj_pose)
        current_pose = self.arm_group.get_current_pose()
        self.Obj_pose.pose.orientation = current_pose.pose.orientation

        obj_x = self.Obj_pose.pose.position.x
        self.Obj_pose.pose.position.x = current_pose.pose.position.x

        self.arm_group.set_pose_target(self.Obj_pose.pose)
        self.arm_group.go()

        self.Obj_pose.pose.position.x = obj_x - 0.2
        self.plan_cartesian_path(self.Obj_pose.pose)

        self.Obj_pose.pose.position.x += 0.05
        self.plan_cartesian_path(self.Obj_pose.pose)

    def move_obj(self):

        self.Obj_pose.pose.position.x = 0.3
        self.plan_cartesian_path(self.Obj_pose.pose)
        self.goSP()
        self.gripper_move(0.040)
        #self.gazeboDetach()

    def main_loop(self):
        try:
            self.gripper_move(0.050)
            self.find_enable = True
            rospy.sleep(3)  # 2. (3s)
            if self.find_enable == False:
                self.goSP()  
                self.grasp_obj()
                rospy.sleep(1)
                self.gripper_move(0.036)
                
                self.move_obj()
                self.Obj_pose.pose.position.x = 0
            else:
                rospy.logwarn('cant find object')

        except Exception as e:
            rospy.logerr(str(e))


def main():
    rospy.init_node('grasp_demo', anonymous=True)
    rospy.loginfo('Start Grasp Demo')
    moverobot = MoveRobot()
    while(not rospy.is_shutdown()):
        moverobot.main_loop()


if __name__ == "__main__":

    main()

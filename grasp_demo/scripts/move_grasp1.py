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
#from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse


class MoveRobot():
    def __init__(self):

        # 初始化 planning group
        self.robot = moveit_commander.robot.RobotCommander()
        self.arm_group = moveit_commander.move_group.MoveGroupCommander(
            "arm_with_torso")

        self.gripper_group = moveit_commander.move_group.MoveGroupCommander(
            "gripper")

        # 设置机械手臂的速度和加速度
        self.arm_group.set_max_acceleration_scaling_factor(1)
        self.arm_group.set_max_velocity_scaling_factor(1)

        # 物体的位置
        self.Obj_pose = PoseStamped()
        self.Obj_pose.pose.position.x = 0
        self.find_enable = False
        self.Obj_class = ''

        # 物体吸附接口

        #self.attach_srv = rospy.ServiceProxy('/link_attacher_node/attach',
                                               #Attach)
        #self.attach_srv.wait_for_service()

        #self.detach_srv = rospy.ServiceProxy('/link_attacher_node/detach',
                                             #Attach)
        #self.detach_srv.wait_for_service()
        
        self.obj_pose_sub = Subscriber(
            "/objection_position_pose", Pose)

        self.yolo_sub = Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes)

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
        moveit_commander.roscpp_initializer.roscpp_shutdown()

    def gripper_move(self, width):
        gripper_joint_values = self.gripper_group.get_current_joint_values()
        gripper_joint_values[0] = width
        gripper_joint_values[1] = width


        self.gripper_group.set_joint_value_target(gripper_joint_values)
        self.gripper_group.go()


  
    def plan_cartesian_path(self, pose):
        """
            笛卡尔路径规划

            Parameters:
                pose - 目标pose

            Returns:
                None
            """
        waypoints = []
        #waypoints.append(self.arm_group.get_current_pose().pose)
        waypoints.append(pose)

        # 设置机器臂当前的状态作为运动初始状态
        self.arm_group.set_start_state_to_current_state()

        # 计算轨迹
        (plan, fraction) = self.arm_group.compute_cartesian_path(
            waypoints,   # waypoint poses，路点列表，这里是5个点
            0.01,        # eef_step，终端步进值，每隔0.01m计算一次逆解判断能否可达
            0.0,         # jump_threshold，跳跃阈值，设置为0代表不允许跳跃
            True)        # avoid_collisions，避障规划

        self.arm_group.execute(plan, wait=True)

    def goSP(self):

        self.arm_group.set_joint_value_target([-9.487505139319357e-07, -1.3286165817724758, 0.9411910978981766, -
                                              2.632872053084035, -2.217766946445675, -1.2135210418771862, 1.477427293130634, 1.4762755298156103])

        self.arm_group.go(wait=True)
        kj = self.arm_group.get_current_joint_values()
        print(kj)

    def grasp_obj(self):

        # 去到物体上方位置
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
        
        self.gripper_move(0.010)
        rospy.sleep(1)
        
        touch_links = self.gripper_group.get_link_names()
        self.arm_group.attach_object(self.obj_class, touch_links=touch_links)
   
        
    def move_base_to_goal(self, x, y, theta):
        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        client.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        quat = tf.transformations.quaternion_from_euler(0, 0, theta)
        goal.target_pose.pose.orientation.x = quat[0]
        goal.target_pose.pose.orientation.y = quat[1]
        goal.target_pose.pose.orientation.z = quat[2]
        goal.target_pose.pose.orientation.w = quat[3]

        client.send_goal(goal)
        client.wait_for_result()

    def move_obj(self):

     
        self.move_base_to_goal(1.0, 0.5, 0.0)  # example coordinates
        
        self.gripper_move(0.040)
        

        self.Obj_pose.pose.position.x = 0.3
        self.plan_cartesian_path(self.Obj_pose.pose)
        self.goSP()
        
        
   

    def main_loop(self):
        try:
            self.gripper_move(0.050)
            self.find_enable = True
            rospy.sleep(3)  # 2. 识别当前的抓取位姿态(3s)
            if self.find_enable == False:
                self.goSP()  # 1. 去到预抓取位置
                self.grasp_obj()
                rospy.sleep(1)
                #self.gripper_move(0.036)
              
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

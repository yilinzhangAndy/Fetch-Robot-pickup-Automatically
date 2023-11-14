#!/usr/bin/env python

import rospy
from move_demo.srv import NavigateToPoint
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib

class Navigator:

    def __init__(self):
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.move_base.wait_for_server()

    def handle_navigate_to_point(self, req):
        # Set up the goal
        target = Pose(Point(req.x, req.y, 0.0), Quaternion(0.0, 0.0, req.theta, 1.0))
        goal = MoveBaseGoal()
        goal.target_pose.pose = target
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()

        # Send goal
        self.move_base.send_goal(goal)

        # Wait for result (5 minutes time limit as in your original script)
        finished_within_time = self.move_base.wait_for_result(rospy.Duration(300))

        # Check result
        if not finished_within_time:
            self.move_base.cancel_goal()
            return False
        else:
            state = self.move_base.get_state()
            return state == GoalStatus.SUCCEEDED

def navigate_to_point_server():
    rospy.init_node('navigate_to_point_server')
    navigator = Navigator()
    s = rospy.Service('navigate_to_point', NavigateToPoint, navigator.handle_navigate_to_point)
    rospy.spin()

if __name__ == "__main__":
    navigate_to_point_server()

#ifndef TF_BROADCAST_H
#define TF_BROADCAST_H

#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include "tf2_msgs/TFMessage.h"
#include <tf/transform_broadcaster.h>
#include "yolov5_ros_msgs/BoundingBoxes.h"
#include "grasp_demo/CamToReal.h"
#include "string.h"
#include<algorithm>

using namespace std;

class findObj
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber tf_sub;
    geometry_msgs::PoseStamped Obj_pose;
    double pixel_x, pixel_y, grasp_angle;
    ros::ServiceClient pose_client;

public:
    bool move_finish;
    findObj();
    ~findObj();
    void pose_callback(const yolov5_ros_msgs::BoundingBoxes &yolo_tmp);
};

#endif
#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
import tf
from tf2_ros import TransformBroadcaster, TransformStamped
from darknet_ros_msgs.msg import BoundingBoxes
from grasp_demo.srv import CamToReal, CamToRealResponse, CamToRealRequest

class TfBroadcast:
    def __init__(self):
        rospy.init_node('tf_broadcast')
        self.tf_broadcaster = TransformBroadcaster()
        self.dist_client = rospy.ServiceProxy('cam_to_real', CamToReal)
        self.dist_req = CamToRealRequest()
        self.dist_resp = CamToRealResponse()
        rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.yolo_callback)

    def yolo_callback(self, yolo_tmp):

        transform = TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = 'head_camera_rgb_optical_frame'
        transform.child_frame_id = 'object'

        for bbox in yolo_tmp.bounding_boxes:
          if bbox.Class == "bottle":
            
            
             pixel_x = (bbox.xmin + bbox.xmax) / 2.0
             pixel_y = (bbox.ymin + bbox.ymax) / 2.0

             print (pixel_x)

             self.dist_req.pixel_x = pixel_x
             self.dist_req.pixel_y = pixel_y
             self.dist_resp = self.dist_client.call(self.dist_req)

             print (self.dist_resp.obj_x)

             transform.transform.translation.x = self.dist_resp.obj_x
             transform.transform.translation.y = self.dist_resp.obj_y
             transform.transform.translation.z = self.dist_resp.obj_z

             q = (0, 0, 0, 1)
             transform.transform.rotation.x = 0
             transform.transform.rotation.y = 0
             transform.transform.rotation.z = 0
             transform.transform.rotation.w = 1

             self.tf_broadcaster.sendTransform(transform)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        tf_broadcast = TfBroadcast()
        tf_broadcast.run()
    except rospy.ROSInterruptException:
        pass

<h1 align="center">
  <img src="image/Picture1.png"><br/>Fetch Robot pickup Automatically
</h1>

</h4>

<div align="center">
 
  <a href="https://wiki.ros.org/melodic"><img src="https://img.shields.io/badge/ROS-Melodic-blue.svg"></a>
  <a href="https://wiki.ros.org/noetic"><img src="https://img.shields.io/badge/ROS-Noetic-blue.svg"></a>




<br/>

<img src="https://github.com/yilinzhangAndy/Fetch-Robot-pickup-Automatically/blob/main/image/pickupgit.gif" width="40%" /> 
</div>

## Darknet_Ros Package

<br/>

<div align="center">
  <img src="image/Yolo.jpg" width="30%">
</div>

YOLO in ROS:

Within the ROS framework, the darknet package enables the use of YOLO for real-time object detection in robotics applications.
This integration allows robots to perceive their environment effectively by identifying and locating various objects in their field of vision quickly.

Functionality:

The package typically includes nodes that can subscribe to image data from ROS topics (usually from a camera), process this data using YOLO for object detection, and then publish the results.
The results might include the types of objects detected, their bounding box coordinates in the image, and possibly their class probabilities.



## TransformBroadcaster File

This Python script, written for use in a ROS (Robot Operating System) environment, is designed to broadcast the 3D position of objects detected by a YOLO (You Only Look Once) object detection system integrated with a camera. Here's a summary of its key functionalities:

-  Initialization in ROS:

    The script initializes a ROS node named 'tf_broadcast', setting up the necessary infrastructure for ROS communication.


- Transform Broadcaster:
  
   It creates a TransformBroadcaster object, which is used to publish transformations between different coordinate frames in a ROS network.


- ROS Service Proxy for Coordinate Transformation:
  
  A service proxy for 'cam_to_real' is established, allowing the script to convert pixel coordinates from the camera to real-world 3D coordinates.
- Subscription to YOLO BoundingBoxes:
  
    The node subscribes to the '/yolov3/BoundingBoxes' topic, which provides bounding box data of detected objects from a YOLOv3 object detection model.
-  Processing YOLO Detection Data:
  
    In the yolo_callback function, the script calculates the center of each detected bounding box in pixel coordinates (X, Y).
  
-  Requesting Real-World Coordinates:
  
    These pixel coordinates are sent to the 'cam_to_real' service, which returns the corresponding real-world 3D coordinates (X, Y, Z) of the detected object.
-  Broadcasting Object Position:
  
    The script then broadcasts the 3D position of the detected object relative to the camera frame ('head_camera_rgb_optical_frame') to the ROS network. This is done using the TransformStamped message, which includes   both   translation and rotation data (though the rotation is set to a default value in this script).
-  Continuous Operation:
  
    The run method keeps the node active to continuously receive and process bounding box data from the YOLO detection system.



In essence, this script bridges the gap between high-level object detection using YOLO and the spatial awareness required in robotics, allowing a robot to understand where objects are located in its environment in 3D space, which is crucial for tasks such as navigation, manipulation, or interaction.

## Camera_to_Real  File

This Python script serves as a ROS node that converts camera pixel coordinates into real-world 3D coordinates for robotic applications. Here's a summary of its functionalities:

- CvBridge Integration:

  It utilizes CvBridge to convert ROS image messages into OpenCV image formats, facilitating image processing tasks.
  
- Depth Image Subscription:

  The node subscribes to a ROS topic that publishes depth images, capturing the distance of each pixel from the camera lens.
- Camera Info Subscription:

   It also subscribes to a topic to receive camera calibration information, which is crucial for accurate 3D space mapping.

- Depth Image Callback:

   This function processes the incoming depth image and converts it into a 2D array that represents the depth values of each pixel.
  
- Camera Info Callback:

   The corresponding function stores the camera's intrinsic parameters, necessary for the conversion from 2D image space to 3D world space.
  
- ROS Service for Coordinate Transformation:

    The node offers a service that, when called, computes the real-world coordinates of a point given its pixel coordinates on the image, using the depth data and the camera's intrinsic parameters.
  
- Pixel to World Transformation:

    By using the depth value and camera information, the script can compute the actual 3D position (X, Y, Z) of an object in space, which is essential for robots to interact accurately with their environment.




In essence, this script is an essential component for robotic vision systems that need to interpret visual information for navigation or manipulation tasks in a 3-dimensional space.






## Object_Grasp  File

This Python script is designed for a robotic application using ROS (Robot Operating System). It integrates object detection (using YOLO through the darknet_ros_msgs package), robotic arm manipulation (using MoveIt!), Here's an overview of its functionality:

- Initialization:

  The script initializes a ROS node and creates instances of MoveIt! command groups for controlling a robot's arm and gripper.
  The arm's and gripper's maximum acceleration and velocity scaling factors are set for smooth operation.
  
- Subscriptions and Service Proxies:

  It subscribes to object detection data (/darknet_ros/BoundingBoxes) and object position data (/objection_position_pose).

- Object Detection and Position Processing:

   An approximate time synchronizer is used to synchronize the incoming messages from the YOLO object detection and the object position topics.
The callback function processes these messages to update the object's pose and classification.

- Gripper Control:

    The gripper_move function controls the gripper's opening width, allowing the robot to grasp objects.
  
- Movement and Grasping Logic:

   The main_loop function orchestrates the robot's movements: moving to a pre-grasp position, adjusting its position based on the detected object, and then grasping the object.
The plan_cartesian_path and goSP methods are used for planning and executing movements of the robot's arm.
  
- Error Handling:

    The script includes error handling in its main loop to log exceptions that might occur during execution.

  
- Execution:

  The script runs in a continuous loop, constantly checking for new object detections and performing grasping and moving operations as required.

This script is a typical example of integrating various ROS functionalities, including perception (object detection), planning (MoveIt! for arm and gripper control),  to create a comprehensive robotic application capable of identifying, grasping, and moving objects.

## Move_service  File

This Python script is a ROS (Robot Operating System) node designed to navigate a robot to a specified point. It utilizes ROS's actionlib library and the move_base package to handle navigation tasks. Here's a breakdown of its key components and functionalities:

- Initialization of the Navigator Class:

  The Navigator class initializes an action client for the move_base action server, which is responsible for handling path planning and movement of the robot.
The move_base server is a key component in ROS for navigating robots, providing an interface to specify a goal position and orientation (pose) in the map frame and handling the necessary planning to reach that goal.
  
- Service Definition for Navigation:

   The node defines a ROS service named 'navigate_to_point', which allows other nodes to request navigation to a specific point. The service expects coordinates (x, y) and orientation (theta).

- Navigation Request Handling:

  The handle_navigate_to_point method is the service callback. It receives the navigation request and sets up a MoveBaseGoal with the target position and orientation.
The target pose is created using the requested x, y coordinates and theta (yaw) for orientation. The orientation is represented as a quaternion.

- Sending the Goal to move_base:

    The goal, containing the target pose, is sent to the move_base action server.
The script waits for the result of the navigation action, with a timeout of 300 seconds (5 minutes).
  
- Result Handling:

   After the timeout or upon receiving a result, the script checks the outcome. If the robot doesn't reach the goal within the specified time, it cancels the goal and returns False.
If the robot successfully reaches the goal, the method returns True.
  
- Node and Service Setup:

    In the navigate_to_point_server function, a ROS node named 'navigate_to_point_server' is initialized, and the navigation service is advertised.
The node then enters a spin state, waiting for service calls.

- Execution:

  When the script is run, it starts the navigation server, ready to accept requests to navigate the robot to specified points.

This script is an excellent example of how ROS integrates various subsystems like action servers, services, and navigation stacks to enable autonomous navigation in robots. It allows other components in a ROS-enabled robot to make use of the move_base functionality through a simple service interface, abstracting the complexities of path planning and control.

<img src="https://github.com/yilinzhangAndy/Fetch-Robot-pickup-Automatically/blob/main/image/pickup 1.gif" width="40%" /> 

# Thank you

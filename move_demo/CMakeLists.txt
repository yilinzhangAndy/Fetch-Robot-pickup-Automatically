cmake_minimum_required(VERSION 3.0.2)
project(move_demo)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
  message_runtime
)

add_service_files(
  FILES
  NavigateToPoint.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(

)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
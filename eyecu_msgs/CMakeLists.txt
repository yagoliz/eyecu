cmake_minimum_required(VERSION 2.8.3)
project(eyecu_msgs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

add_message_files(
  DIRECTORY msg
  FILES
  point.msg
  boundingBox.msg
  boundingBoxArray.msg
  DistanceCamera.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)


catkin_package(
  CATKIN_DEPENDS
  message_runtime
  std_msgs
)

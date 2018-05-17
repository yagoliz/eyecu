cmake_minimum_required(VERSION 2.8.3)
project(eyecu)

add_subdirectory($ENV{HOME}/cmake_workspace/dlib-19.10/dlib dlib_build)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
  message_generation
)

find_package(OpenCV REQUIRED)

find_package(Boost REQUIRED COMPONENTS system)

add_message_files(
   FILES
   centroid.msg
   DistanceCamera.msg
 )


## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
 )

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
  INCLUDE_DIRS include
)


include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

 add_executable(eyecu_node src/eyecu_node.cpp)
 target_link_libraries(eyecu_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
  )

  add_executable(eyecu_node_dlib src/eyecu_node_dlib.cpp)
  target_link_libraries(eyecu_node_dlib
     ${catkin_LIBRARIES}
     ${OpenCV_LIBRARIES}
     dlib::dlib
   )

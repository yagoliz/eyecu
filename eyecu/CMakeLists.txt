cmake_minimum_required(VERSION 2.8.3)
project(eyecu)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
  eyecu_msgs
)

find_package(OpenCV 3.4 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(dlib)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs
  INCLUDE_DIRS include
)


include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

 add_executable(eyecu_node src/eyecu_node.cpp)
 target_link_libraries(eyecu_node
    ${OpenCV_LIBRARIES}
    ${catkin_LIBRARIES}
  )

  add_executable(eyecu_node_dlib src/eyecu_node_dlib.cpp)
  target_link_libraries(eyecu_node_dlib
     ${catkin_LIBRARIES}
     ${OpenCV_LIBRARIES}
     ${dlib_LIBRARIES}
   )

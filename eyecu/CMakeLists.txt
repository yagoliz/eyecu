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
  eyecu_msgs
  pcl_conversions
  pcl_ros
  laser_geometry
  message_filters
  visualization_msgs
)

find_package(OpenCV 3 REQUIRED)

find_package(Boost REQUIRED COMPONENTS system)

find_package (PCL REQUIRED)
include_directories(include ${PCL_INCLUDE_DIRS})

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
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
  )

  add_executable(eyecu_node_dlib src/eyecu_node_dlib.cpp)
  target_link_libraries(eyecu_node_dlib
     ${catkin_LIBRARIES}
     ${OpenCV_LIBRARIES}
     dlib::dlib
   )

   add_executable(pointcloud_image_projection src/pointcloud_image_projection.cpp)
   target_link_libraries(pointcloud_image_projection
     ${catkin_LIBRARIES}
     ${OpenCV_LIBRARIES}
   )

   add_executable(laser_image_projection src/laser_image_projection.cpp)
   target_link_libraries(laser_image_projection
     ${catkin_LIBRARIES}
     ${OpenCV_LIBRARIES}
   )

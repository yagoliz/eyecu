cmake_minimum_required(VERSION 3.0)
project(face_tracker_control)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  dynamixel_controllers
  roscpp
  rospy
  message_generation
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  message_filters
)

find_package(Eigen3 REQUIRED NO_MODULE)

#find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)

add_message_files(DIRECTORY msg
   FILES
   centroid.msg
 )


## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs

 )


catkin_package(
  CATKIN_DEPENDS message_runtime dynamixel_controllers roscpp rospy std_msgs

)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_executable(face_tracker_control src/face_tracker_control.cpp)
add_executable(motor_state_broadcaster src/motor_state_broadcaster.cpp)

add_dependencies(face_tracker_control ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(motor_state_broadcaster ${${PROJECT_NAME}_EXPORTED_TARGETS})

target_link_libraries(face_tracker_control ${catkin_LIBRARIES} ${Eigen3_LIBRARIES})
target_link_libraries(motor_state_broadcaster ${catkin_LIBRARIES} ${Eigen3_LIBRARIES})

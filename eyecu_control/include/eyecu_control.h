/* Code developed by Yago Lizarribar at MIT Media Lab based on Lentin Joseph's
 * face tracker control
 *
 * This code has been modified to move 2 servos at the same time
 * For any questions email: yagol@mit.edu
*/

#define PI 3.14159265359

// Standard Libraries
#include <cmath>
#include <iostream>

// Include other libraries
#include <Eigen/Dense>

// Ros header files
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>

// Custom msgs
#include <eyecu/DistanceCamera.h>

// Eigen namespace
using namespace Eigen;


class FaceTracker {

  // ROS variables
  ros::NodeHandle nh;

  //Tracker parameters
  float servomaxx, servomaxy;
  bool debug;
  // Topics and frames (these are parameters too)
  std::string face_distance_topic, pan_controller_command, tilt_controller_command;
  std::string source_frame, target_frame;

  // Publishers & their variables
  ros::Publisher pan_control ; std_msgs::Float64 pan_pose ;
  ros::Publisher tilt_control; std_msgs::Float64 tilt_pose;
  ros::Publisher debug_pub   ; std_msgs::Float64 angle    ;

  // Subscriber
  ros::Subscriber face_subscriber;

  // Vectors for transformations
  geometry_msgs::TransformStamped transformStamped;
  tf2::Stamped<Eigen::Vector3d> t_in;
  tf2::Stamped<Eigen::Vector3d> t_out;

  public:

    FaceTracker();
    ~FaceTracker(){;}

    void track_face(double x, double y, double z);
    void face_callback(const eyecu::DistanceCamera::ConstPtr& msg);

};

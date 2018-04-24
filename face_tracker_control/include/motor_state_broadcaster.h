/* Code developed by Yago Lizarribar at MIT Media Lab based on Lentin Joseph's
 * face tracker control
 *
 * This code has been modified to move 2 servos at the same time
 * For any questions email: yagol@mit.edu
*/

// Standard Libraries
#include <cmath>
#include <iostream>

// Include other libraries
#include <Eigen/Dense>

// Ros header files
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <dynamixel_msgs/JointState.h>

// Message filters
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

typedef message_filters::sync_policies::ApproximateTime<dynamixel_msgs::JointState, dynamixel_msgs::JointState> SyncPolicy;

class MotorStateBroadcaster {

  ros::NodeHandle nh;
  message_filters::Subscriber<dynamixel_msgs::JointState> pan_subscriber;
  message_filters::Subscriber<dynamixel_msgs::JointState> tilt_subscriber;

  geometry_msgs::TransformStamped transform_pan, transform_tilt;

  std::string pan_topic, tilt_topic;
  std::string eye_link, tilt_link, gaze_link;

  message_filters::Synchronizer<SyncPolicy> *sync;

  // Variables for
  float tilt_x, tilt_y, tilt_z;
  float gaze_x, gaze_y, gaze_z;

public:
  MotorStateBroadcaster();
  ~MotorStateBroadcaster() {;}

  void callback(const dynamixel_msgs::JointStateConstPtr& pan, const dynamixel_msgs::JointStateConstPtr& tilt);

};

/*
 * Copyright (C) 2017, Lentin Joseph and Qbotics Labs Inc.

 * Email id : qboticslabs@gmail.com

 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.


* This code will subscriber centroid of face and move dynamixel : face_tracker_controller.cpp

*/

/*
Value to dynamixel controller

Center = 0  // Dynamixel value = 512
Right = 1   // Dynamixel value = 708
Left = -1   // Dyanmixel value = 316

Differenece is 196 unit from center in Dynamixel
Optimum range = -0.5 to 0.5
*/

/*
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
#include <face_tracker_pkg/DistanceCamera.h>

// Eigen namespace
using namespace Eigen;


//Tracker parameters
float servomaxx, servomaxy;

std_msgs::Float64 pan_pose ;
std_msgs::Float64 tilt_pose;

std_msgs::Float64 angle;

ros::Publisher pan_control;
ros::Publisher tilt_control;
ros::Publisher debug_pub;

std::string face_distance_topic, pan_controller_command, tilt_controller_command;
std::string source_frame, target_frame;

bool debug;


// Vectors for transformations
geometry_msgs::TransformStamped transformStamped;
tf2::Stamped<Eigen::Vector3d> t_in;
tf2::Stamped<Eigen::Vector3d> t_out;


// Controller functions
void track_face(double x, double y, double z)
{
  // ROS_INFO("X: %f, Y: %f, Z: %f", x, y, z);

  double phi   = PI/2-atan2(x,y);
  double theta = -atan2(z,sqrt(pow(x,2)+pow(y,2)));

  // int sign_phi = phi/abs(phi);
  // int sign_theta = theta/abs(theta);
  //
  // phi = sign_phi * std::min(abs(phi), servomaxx);
  // theta = sign_theta * std::min(abs(theta), servomaxy);

	pan_pose.data = phi;
  tilt_pose.data = theta;

  if (debug){
  	// printf("phi:%f, theta: %f \n", phi, theta);
  	debug_pub.publish(transformStamped);
  }

  pan_control .publish(pan_pose);
  tilt_control.publish(tilt_pose);

}

//Callback of the topic /numbers
void face_callback(const face_tracker_pkg::DistanceCamera::ConstPtr& msg)
{
	// ROS_INFO("Recieved X = [%f], Y = [%f], Z = [%f]", msg->X, msg->Y, msg->Z);

  t_in(0) =  msg->Z/100;
  t_in(1) = -msg->X/100;
  t_in(2) = -msg->Y/100;

  std::cout << t_in << std::endl;

  tf2::doTransform(t_in, t_out, transformStamped);

  std::cout << t_out << std::endl;

	//Calling track face function
	track_face(t_out(0), t_out(1), t_out(2));

}

// Main function
int main(int argc, char **argv)
{
  //Loading servo configurations of the dynamixel tracker
	//Initializing ROS node with a name of demo_topic_subscriber
	ros::init(argc, argv,"face_tracker_controller");

  //Created a nodehandle object
  ros::NodeHandle nh;

  // Loading default values
  face_distance_topic = "/face_distance";
  pan_controller_command = "pan_controller/command";
  tilt_controller_command = "tilt_controller/command";
  target_frame = "right_eye_link";
  source_frame = "camera_link";
  servomaxx = 1.0;
  servomaxy = 1.0;
  debug = true;

  try
  {
    nh.getParam("face_distance_topic" , face_distance_topic );
    nh.getParam("pan_controller_command" , pan_controller_command );
    nh.getParam("tilt_controller_command", tilt_controller_command);
    nh.getParam("servomaxx", servomaxx);
    nh.getParam("servomaxy", servomaxy);
    nh.getParam("debug", debug);
    nh.getParam("target_frame", target_frame);
    nh.getParam("source_frame", source_frame);

    ROS_INFO("Successfully Loaded tracking parameters");
  }
  catch(int e)
  {
    ROS_WARN("Parameters are not properly loaded from file, loading defaults");
  }

  std::cout << source_frame << std::endl;
  // Transform variables
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

	//Create subscriber and publisher objects
	ros::Subscriber number_subscriber = nh.subscribe(face_distance_topic, 10 ,face_callback);
  pan_control  = nh.advertise<std_msgs::Float64>(pan_controller_command, 1);
  tilt_control = nh.advertise<std_msgs::Float64>(tilt_controller_command, 1);

	debug_pub = nh.advertise<geometry_msgs::TransformStamped>("/debug", 1);

	servomaxx = 1.0;   //max degree servo horizontal (x) can turn
	servomaxy = 1.0;


	//Sending initial pose
  pan_pose.data  = 0.0;
  tilt_pose.data = 0.0;

  pan_control .publish(pan_pose);
  tilt_control.publish(tilt_pose);


  try {
    transformStamped = tfBuffer.lookupTransform(target_frame, source_frame, ros::Time(1), ros::Duration(5));
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("%s",ex.what());
    ros::Duration(1.0).sleep();
  }


	//Spinning the node
	ros::spin();
	return 0;
}

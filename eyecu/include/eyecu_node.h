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
* This code will track the faces using ROS
*/

/*
 * This code has been modified to calculate the distance of the face to the camera frame
 * and return the position in spherical coordinates.

 * For any questions email: yagol@mit.edu
*/

// Standard libraries
#include <iostream>
#include <string>
#include <stdio.h>
#include <cmath>

//Open-CV headers
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/objdetect.hpp"

//ROS headers
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/camera_subscriber.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>

//Centroid message headers
#include <eyecu_msgs/DistanceCamera.h>

using namespace std;
using namespace cv;

class Face_Detector
{

  // ROS variables
  ros::NodeHandle nh_;

  image_transport::ImageTransport it_;
  image_transport::CameraSubscriber camera_sub_;
  image_transport::Publisher image_pub_;

  ros::Publisher face_distance_pub;
  eyecu_msgs::DistanceCamera face_distance;

  // Topic names and cascade file path
  string base_input_topic, output_image_topic, haar_file_face;

  // Camera values
  int face_tracking, center_offset, screenmaxx, screenmaxy;

  Point2f center, center_face, opposite;

  bool display_original_image, display_tracking_image;

  float fx, fy, cx, cy;
  float delta_u, delta_v;

public:
  Face_Detector();

  ~Face_Detector();

  void imageCb(const sensor_msgs::ImageConstPtr& img, const sensor_msgs::CameraInfoConstPtr& Cinfo);

  void detectAndDraw( Mat& img, CascadeClassifier& cascade);

};

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

// dlib headers
#include <dlib/opencv.h>
#include <dlib/image_processing/frontal_face_detector.h>
#include <dlib/image_processing/render_face_detections.h>
#include <dlib/image_processing.h>
#include <dlib/gui_widgets.h>

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
// using namespace dlib;

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
  string base_input_topic, output_image_topic;

  // dlib variables
  dlib::frontal_face_detector detector;
  dlib::image_window win;

  // Camera values
  int screenmaxx, screenmaxy;

  Point2f center, center_face, opposite;

  bool display_original_image, display_tracking_image;

  float fx, fy, cx, cy;
  float delta_u, delta_v;

public:
  Face_Detector();

  ~Face_Detector();

  void imageCb(const sensor_msgs::ImageConstPtr& img, const sensor_msgs::CameraInfoConstPtr& Cinfo);

  void detectAndDraw( Mat& img);

};

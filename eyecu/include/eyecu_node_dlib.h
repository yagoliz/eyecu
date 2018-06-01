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
#include <image_transport/subscriber.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

//Custom message headers
#include <eyecu_msgs/boundingBox.h>
#include <eyecu_msgs/boundingBoxArray.h>

using namespace std;
using namespace cv;
// using namespace dlib;

class Face_Detector
{

  // ROS variables
  ros::NodeHandle nh_;

  image_transport::ImageTransport it_;
  image_transport::Subscriber camera_sub_;

  ros::Publisher bbox_pub;

  // Topic names and cascade file path
  string base_input_topic;

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

  void imageCb(const sensor_msgs::ImageConstPtr& img);

  void detectAndDraw( Mat& img);

};

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
 * and return the position in camera coordinates:
 * X+: Pointing to the right of the image
 * Y+: Pointing downwards
 * Z+: Pointing to the screen
 * For any questions email: yagol@mit.edu
*/

// Average face dimensions: https://en.wikipedia.org/wiki/Human_head
#define FACE_WIDTH 13.9 // cm
#define FACE_HEIGHT 22.5 // cm

#include <eyecu_node.h>

//OpenCV window name
static const std::string OPENCV_WINDOW = "raw_image_window";
static const std::string OPENCV_WINDOW_1 = "face_detector";


using namespace std;
using namespace cv;

Face_Detector::Face_Detector(): it_(nh_)
{
  //Loading Default values

  base_input_topic = "/usb_cam/image_raw";
  haar_file_face = "/home/yago/opencv/data/haarcascades/haarcascade_frontalface_alt.xml";
  face_tracking = 1;
  display_original_image = false;
  display_tracking_image = true;
  center_offset = 100;

  //Accessing parameters from track.yaml
  try{
    nh_.getParam("base_input_topic", base_input_topic);
    nh_.getParam("face_detected_image_topic", output_image_topic);
    nh_.getParam("haar_file_face", haar_file_face);
    nh_.getParam("face_tracking", face_tracking);
    nh_.getParam("display_original_image", display_original_image);
    nh_.getParam("display_tracking_image", display_tracking_image);
    nh_.getParam("center_offset", center_offset);

    ROS_INFO("Successfully Loaded tracking parameters");
  }

  catch(int e)
  {
    ROS_WARN("Parameters are not properly loaded from file, loading defaults");
  }

  // CamInfo = ros::topic::waitForMessage("/usb_cam/camera_info", ros::Duration(2));

  // Subscribe to input video feed and publish output video feed
  camera_sub_ = it_.subscribeCamera(base_input_topic, 1,
    &Face_Detector::imageCb, this);
  image_pub_ = it_.advertise(output_image_topic, 1);

  face_distance_pub = nh_.advertise<eyecu_msgs::DistanceCamera>("/face_distance",10);
}

Face_Detector::~Face_Detector()
{
  if( display_original_image == 1 or display_tracking_image == 1)
    cv::destroyAllWindows();
}



void Face_Detector::imageCb(const sensor_msgs::ImageConstPtr& img, const sensor_msgs::CameraInfoConstPtr& Cinfo)
{

  cv_bridge::CvImagePtr cv_ptr;

  screenmaxx = Cinfo->width;
  screenmaxy = Cinfo->height;

  center.x = (double)screenmaxx/2;
  center.y = (double)screenmaxy/2;

  fx = Cinfo->K[0];
  cx = Cinfo->K[2];
  fy = Cinfo->K[4];
  cy = Cinfo->K[5];

  try
  {
    cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }


  // string cascadeName = haar_file_face;
  CascadeClassifier cascade;
  if( !cascade.load( haar_file_face ) )
  {
    cerr << "ERROR: Could not load classifier cascade" << endl;
    return;
  }


  if (display_original_image){
    imshow("Original Image", cv_ptr->image);
  }

  detectAndDraw( cv_ptr->image, cascade );

  image_pub_.publish(cv_ptr->toImageMsg());

  waitKey(30);
}

void Face_Detector::detectAndDraw( Mat& img, CascadeClassifier& cascade)
{
  double t = 0;
  double scale = 1;
  vector<Rect> faces;
  const static Scalar colors[] =
  {
      Scalar(255,0,0),
      Scalar(255,128,0),
      Scalar(255,255,0),
      Scalar(0,255,0),
      Scalar(0,128,255),
      Scalar(0,255,255),
      Scalar(0,0,255),
      Scalar(255,0,255)
  };
  Mat gray, smallImg;

  cvtColor( img, gray, COLOR_BGR2GRAY );
  double f = 1 / scale ;
  resize( gray, smallImg, Size(), f, f, INTER_LINEAR );
  equalizeHist( smallImg, smallImg );

  t = (double)cvGetTickCount();
  cascade.detectMultiScale( smallImg, faces,
      1.1, 15, 0
      |CASCADE_SCALE_IMAGE,
      Size(30, 30) );

  t = (double)cvGetTickCount() - t;

  // for ( size_t i = 0; i < faces.size(); i++ )
  if (faces.size() > 0)
  {
      Rect r = faces[0];
      Scalar color = colors[0];

      center_face.x = r.x + r.width /2;
      center_face.y = r.y + r.height/2;

      opposite.x = r.x + r.width ;
      opposite.y = r.y + r.height;

      face_distance.Z = (pow(fx*FACE_WIDTH, 2) + pow(fy*FACE_HEIGHT, 2)) /
          (((opposite.x-r.x)*fx*FACE_WIDTH) + ((opposite.y-r.y)*fy*FACE_HEIGHT));

      face_distance.X = (center_face.x - center.x) * face_distance.Z/fx;
      face_distance.Y = (center_face.y - center.y) * face_distance.Z/fy;

      face_distance_pub.publish(face_distance);

      rectangle(img, Point(r.x, r.y), opposite, color, 1, CV_AA);
  }
  else
  {
    face_distance.X = 0;
    face_distance.Y = 0;
    face_distance.Z = 0;
  }

  //Adding lines and left | right sections

  Point pt1, pt2,pt3,pt4,pt5,pt6;

  //Center line
  pt1.x = screenmaxx / 2;
  pt1.y = 0;

  pt2.x = screenmaxx / 2;
  pt2.y = screenmaxy;


  //Left center threshold
  pt3.x = (screenmaxx / 2) - center_offset;
  pt3.y = 0;

  pt4.x = (screenmaxx / 2) - center_offset;
  pt4.y = screenmaxy;

  //Right center threshold
  pt5.x = (screenmaxx / 2) + center_offset;
  pt5.y = 0;

  pt6.x = (screenmaxx / 2) + center_offset;
  pt6.y = screenmaxy;


  line(img,  pt1,  pt2, Scalar(0, 0, 255),0.2);
  line(img,  pt3,  pt4, Scalar(0, 255, 0),0.2);
  line(img,  pt5,  pt6, Scalar(0, 255, 0),0.2);


  putText(img, "Left", cvPoint(50,240), FONT_HERSHEY_SIMPLEX, 1, cvScalar(255,0,0), 2, CV_AA);
  putText(img, "Center", cvPoint(280,240), FONT_HERSHEY_SIMPLEX, 1, cvScalar(0,0,255), 2, CV_AA);
  putText(img, "Right", cvPoint(480,240), FONT_HERSHEY_SIMPLEX, 1, cvScalar(255,0,0), 2, CV_AA);

  if (display_tracking_image)
  {
  	imshow( "Face tracker", img );
  }

}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "Face tracker");
  Face_Detector ic;
  ros::spin();
  return 0;
}

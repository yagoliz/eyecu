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
  display_original_image = false;
  display_tracking_image = true;

  //Accessing parameters from track.yaml
  try{
    nh_.getParam("base_input_topic", base_input_topic);
    nh_.getParam("haar_file_face", haar_file_face);
    nh_.getParam("display_original_image", display_original_image);
    nh_.getParam("display_tracking_image", display_tracking_image);

    ROS_INFO("Successfully Loaded tracking parameters");
  }

  catch(int e)
  {
    ROS_WARN("Parameters are not properly loaded from file, loading defaults");
  }

  // Subscribe to input video feed and publish output video feed
  camera_sub_ = it_.subscribe(base_input_topic, 1,
    &Face_Detector::imageCb, this);

  bbox_pub = nh_.advertise<eyecu_msgs::boundingBoxArray>("/bounding_boxes",10);
}

Face_Detector::~Face_Detector()
{
  if( display_original_image == 1 or display_tracking_image == 1)
    cv::destroyAllWindows();
}



void Face_Detector::imageCb(const sensor_msgs::ImageConstPtr& img)
{

  cv_bridge::CvImagePtr cv_ptr;

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
    eyecu_msgs::boundingBox bbox;
    eyecu_msgs::boundingBoxArray bboxes;

    for (int i = 0; i < faces.size(); i++) {
      Rect r = faces[i];

      bbox.name = "face";
      bbox.pointA.x = r.x;
      bbox.pointA.y = r.y;
      bbox.pointB.x = r.x + r.width ;
      bbox.pointB.y = r.y + r.height;

      bboxes.boundingBoxes.push_back(bbox);

      rectangle(img, Point(r.x, r.y), Point(r.x+r.width,r.y+r.height), colors[0], 1, CV_AA);
    }

    bbox_pub.publish(bboxes);
  }

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

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

#include <face_tracker_node_dlib.h>

using namespace std;
using namespace cv;

Face_Detector::Face_Detector(): it_(nh_)
{
  //Loading Default values

  base_input_topic = "/usb_cam/image_raw";
  display_original_image = false;
  display_tracking_image = true;
  center_offset = 100;

  //Accessing parameters from track.yaml
  try
  {
    nh_.getParam("base_input_topic", base_input_topic);
    nh_.getParam("face_detected_image_topic", output_image_topic);
    nh_.getParam("display_original_image", display_original_image);
    nh_.getParam("display_tracking_image", display_tracking_image);

    ROS_INFO("Successfully Loaded tracking parameters");
  }

  catch(int e)
  {
    ROS_WARN("Parameters are not properly loaded from file, loading defaults");
  }

  // Start dlib detector
  detector = dlib::get_frontal_face_detector();

  // Subscribe to input video feed and publish output video feed
  camera_sub_ = it_.subscribeCamera(base_input_topic, 1,
    &Face_Detector::imageCb, this);

  face_distance_pub = nh_.advertise<face_tracker_pkg::DistanceCamera>("/face_distance",10);
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

  if (display_original_image)
  {
    imshow("Original Image", cv_ptr->image);
  }

  detectAndDraw(cv_ptr->image);

  waitKey(30);
}

void Face_Detector::detectAndDraw( Mat& img)
{
  double t = 0;
  double scale = 1;
  vector<dlib::rectangle> faces;

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

  dlib::cv_image<dlib::bgr_pixel> cimg(img);
  faces = detector(cimg);

  // for ( size_t i = 0; i < faces.size(); i++ )
  if (faces.size() > 0)
  {
      dlib::rectangle r = faces[0];
      Scalar color = colors[0];

      center_face.x = r.left() + r.width ()/2;
      center_face.y = r.top () + r.height()/2;

      opposite.x = r.left() + r.width ();
      opposite.y = r.top () + r.height();

      face_distance.Z = (pow(fx*FACE_WIDTH, 2) + pow(fy*FACE_HEIGHT, 2)) /
          (((opposite.x-r.)*fx*FACE_WIDTH) + ((opposite.y-r.top())*fy*FACE_HEIGHT));

      face_distance.X = (center_face.x - center.x) * face_distance.Z/fx;
      face_distance.Y = (center_face.y - center.y) * face_distance.Z/fy;

      face_distance_pub.publish(face_distance);

  }
  else
  {
    face_distance.X = 0;
    face_distance.Y = 0;
    face_distance.Z = 0;
  }

  if (display_tracking_image)
  {
    win.clear_overlay();
    win.set_image(cimg);
    win.add_overlay(faces, dlib::rgb_pixel(0, 0, 255));
  }
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "Face tracker");
  Face_Detector ic;
  ros::spin();
  return 0;
}

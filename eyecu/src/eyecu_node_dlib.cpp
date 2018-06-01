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

#include <eyecu_node_dlib.h>

using namespace std;
using namespace cv;

Face_Detector::Face_Detector(): it_(nh_)
{
  //Loading Default values

  base_input_topic = "/usb_cam/image_raw";
  display_original_image = false;
  display_tracking_image = true;

  //Accessing parameters from track.yaml
  try
  {
    nh_.getParam("base_input_topic", base_input_topic);
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
    eyecu_msgs::boundingBox bbox;
    eyecu_msgs::boundingBoxArray bboxes;

    for (int i = 0; i < faces.size(); i++) {
      dlib::rectangle r = faces[i];

      bbox.name = "face";
      bbox.pointA.x = r.left  ();
      bbox.pointA.y = r.top   ();
      bbox.pointB.x = r.right ();
      bbox.pointB.y = r.bottom();

      bboxes.boundingBoxes.push_back(bbox);
    }

    bbox_pub.publish(bboxes);

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

#include <laser_projection/laser_image_projection.h>

LaserImageProjection::LaserImageProjection(ros::NodeHandle n):
  n_(n),
  laser_sub_(n_, "scan", 10),
  image_sub_(n_, "/front_camera/image_raw", 10),
  cinfo_sub_(n_, "/front_camera/camera_info", 10)
{
  // First we load the private parameters
  ros::NodeHandle nh_private("~");
  nh_private.param<bool>("display", display_, false);
  nh_private.param<bool>("pub_marks", pub_marks_, false);
  nh_private.param<bool>("pub_result", pub_result_, true);
  nh_private.param<bool>("display", display_, false);

  // We initialize the camera matrix
  initMatrix();

  // Start image transport "nodehandle"
  image_transport::ImageTransport it(n);

  // Start publishers
  if (pub_marks_)
  {
    marker_pub = n.advertise<visualization_msgs::Marker>("object_show", 1);
  }
  if (pub_result_)
  {
    result_pub_ = it.advertise("result_image", 1);
  }

  face_distance_pub_ = n.advertise<eyecu_msgs::DistanceCamera>("/face_distance", 1);

  image_pub_ = it.advertise("pub_image", 1);

  face_distance_.X = 0;
  face_distance_.Y = 0;
  face_distance_.Z = 1;

  // Initialize sync policy and start callback
  first_time_ = true;
  sync_ = new message_filters::Synchronizer<SyncPolicy>
    ( SyncPolicy(10), laser_sub_, image_sub_, cinfo_sub_);

  sync_->registerCallback(std::tr1::bind(&LaserImageProjection::callbackMethod,
                          this, std::tr1::placeholders::_1, std::tr1::placeholders::_2,
                          std::tr1::placeholders::_3 ));
}

void LaserImageProjection::initMatrix()
{
  // Initialize projection matrix (3D -> 2D)
  camProjection_matrix_.resize(3, 4);
  camProjection_matrix_.setZero();
}

void LaserImageProjection::setTransformMatrix(const sensor_msgs::CameraInfoConstPtr& cinfo_in, tf::StampedTransform c_tf)
{
  // This function is called once to calculate the actual projection matrix
  // Declare matrices and variables
  MatrixXd projection_matrix;
  MatrixXd camTranslate_matrix;
  MatrixXd camRotate_matrix;
  double roll, pitch, yaw;

  // Initialize matrices
  projection_matrix.resize(3, 4);
  projection_matrix.setZero();
  camTranslate_matrix.resize(4, 4);
  camTranslate_matrix.setIdentity();
  camRotate_matrix.resize(4, 4);
  camRotate_matrix.setIdentity();

  // Add values to the projection matrix
  for(int i = 0; i < 3; i++)
  {
    for(int j = 0; j < 4; j++)
    {
      projection_matrix(i, j) = cinfo_in->P[4*i+j];
    }
  }

  // Variables to store the Euler angles of the rotation matrix
  tf::Matrix3x3 m(c_tf.getRotation());
  m.getRPY(roll, pitch, yaw);

  camTranslate_matrix(0, 3) =  c_tf.getOrigin().y();
  camTranslate_matrix(1, 3) =  c_tf.getOrigin().z();
  camTranslate_matrix(2, 3) = -c_tf.getOrigin().x();
  camRotate_matrix(0, 0) =  cos(yaw);
  camRotate_matrix(0, 2) =  sin(yaw);
  camRotate_matrix(2, 0) = -sin(yaw);
  camRotate_matrix(2, 2) =  cos(yaw);

  // Computation of projection matrix
  camProjection_matrix_ = projection_matrix * camTranslate_matrix * camRotate_matrix;
}

void LaserImageProjection::laserProjection( pcl::PointCloud<pcl::PointXYZ> &cloud,
                      MatrixXd &imagePoints)
{
  // This funtion projects the pointcloud to the  image
  int n = cloud.width * cloud.height;
  int count = 0;
  imagePoints.resize(4, n);
  MatrixXd scanPoints(4, n);
  for (int i = 0; i < n; i++)
  {
    scanPoints(0, i) = -cloud.points[i].y;
    scanPoints(1, i) = -cloud.points[i].z;
    scanPoints(2, i) =  cloud.points[i].x;
    scanPoints(3, i) = 1;
  }
  imagePoints =  camProjection_matrix_ * scanPoints;
}

void LaserImageProjection::callHeatMap(float value, float min, float max, int &r, int &g, int &b )
{
  // This function colors points according to distance
  float ratio = 2.0 * (float)(value-min) / (float)(max - min);
  b = (int) 255*(1 - ratio);
  if (b < 0) b = 0;
  r = (int) 255*(ratio - 1);
  if (r < 0) r = 0;
  g = 255 - b - r;
}

void LaserImageProjection::putBoxDistance(int &x, int &y, double &distance, pcl::PointXYZ &point, std::vector<DepthBox> &list)
{
  // This function calculates wether a point is inside a bounding box
  for(int i = 0; i < list.size(); i++)
  {
    if (   x > list[i].rect.x + list[i].rect.width/4
        && x < list[i].rect.x + list[i].rect.width - list[i].rect.width/4
        && y > list[i].rect.y + list[i].rect.height/4
        && y < list[i].rect.y + list[i].rect.height - list[i].rect.height/4)
    {
      if(list[i].dis == -1)
      {
        list[i].dis = distance;
        list[i].point = point;
        face_distance_.X = -point.y;
        face_distance_.Y = -point.z;
        face_distance_.Z =  point.x;
      }
      else
      {
        // nearest point
        if(distance < list[i].dis)
        {
          list[i].dis = distance;
          list[i].point = point;
          face_distance_.X = -point.y;
          face_distance_.Y = -point.z;
          face_distance_.Z =  point.x;
        }
      }
    }
  }
}

void LaserImageProjection::ProjectImage(MatrixXd &imagePoints,
                      const sensor_msgs::CameraInfoConstPtr& cinfo_in,
                      pcl::PointCloud<pcl::PointXYZ> &cloud,
                      cv::Mat &img)
{
  // This function compares the 3D and projected points to return distance of objects

  int x, y, d;
  int r, g, b;
  int x_shift, y_shift;
  // w is distance to camera
  float w;
  for (int i = 0; i < imagePoints.cols(); i++){
    w = imagePoints(2, i);
    if (w > 0)
    {
      x = (int)(imagePoints(0, i) / w);
      y = (int)(imagePoints(1, i) / w);
      if (x >= 0 && x < cinfo_in->width && y >= 0 && y < cinfo_in->height)
      {
        x_shift = x;
        y_shift = y;

        // Calculate the distance in 3D and call function to put the distance of the object
        // in case there is any
        double distance = sqrt(pow(cloud.points[i].x, 2) + pow(cloud.points[i].y, 2));
        putBoxDistance(x_shift , y_shift, distance, cloud.points[i], box_list_);

        // This function changes the color of pointcloud according to its distance
        callHeatMap(distance, HEAT_MIN, HEAT_MAX, r, g, b);
        cv::circle( img, cv::Point(x_shift, y_shift), 3, cv::Scalar(r, g, b), -1, 8 );
      }
    }
  }
  face_distance_pub_.publish(face_distance_);
}

void LaserImageProjection::putBoxOnRVIZ(int id, pcl::PointXYZ &point, std::string &str)
{
  marker.header.frame_id = "mark";
  marker.header.stamp = ros::Time::now();
  marker.ns = "object";
  marker.id = id;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = point.x;
  marker.pose.position.y = point.y;
  marker.pose.position.z = point.z;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.5;
  marker.scale.y = 0.5;
  marker.scale.z = 2.0;
  if(str.compare("person")==0)
  {
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
  }
  else if(str.compare("tvmonitor")==0)
  {
    marker.type = visualization_msgs::Marker::CUBE;
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
  }
  else
  {
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
  }

  marker.color.a = 1.0;
  marker.lifetime = ros::Duration(0.1);
  marker_pub.publish(marker);
}

void LaserImageProjection::drawBoundingBox(cv::Mat& img, std::vector<DepthBox> &list)
{
  for (int i = 0; i < list.size(); i++)
  {
    // if (display_ || pub_result_)
    // {
    //   cv::rectangle(img, list[i].rect, cv::Scalar(255, 255, 0), 2, 4, 0);
    //   cv::putText(img, list[i].name, cv::Point(list[i].rect.x, list[i].rect.y - 5),
    //               cv::FONT_HERSHEY_SIMPLEX, 1,cv::Scalar(100, 255, 0), 3, 8);
    // }
    if(list[i].dis != -1 )
    {
      if (display_ || pub_result_)
      {
        std::string s = boost::lexical_cast<std::string>(list[i].dis).substr(0, 5) + " m";
        cv::putText(img, s, cv::Point(list[i].rect.x + 5, list[i].rect.y + 30),
                    cv::FONT_HERSHEY_SIMPLEX, 1,cv::Scalar(0, 0, 255), 3, 8);
      }
      if(pub_marks_)
      {
        putBoxOnRVIZ(i, list[i].point, list[i].name);
      }
    }
  }
}

void LaserImageProjection::callbackMethod ( const sensor_msgs::LaserScanConstPtr& scan_in,
                      const sensor_msgs::ImageConstPtr& img_in,
                      const sensor_msgs::CameraInfoConstPtr& cinfo_in )
{
  pcl::PCLPointCloud2 cloud2_pcl;
  PCLPointCloud cloud_pcl;
  cv_bridge::CvImagePtr cv_ptr;
  MatrixXd imagePoints;
  if (first_time_)
  {
    ros::Time now = ros::Time::now();
    tf::TransformListener tf_listener;
    tf::StampedTransform c_tf;
    try
    {
      tf_listener.waitForTransform("/base_link", "/camera_link",
                                    now, ros::Duration(1.0));
      tf_listener.lookupTransform("/base_link", "/camera_link",
                                    now, c_tf);
    }
    catch (tf::TransformException& e)
    {
      std::cout << "[ERROR]" << e.what() << std::endl;
      return;
    }
    setTransformMatrix(cinfo_in, c_tf);
    std::cout << "Matrix initial!" << std::endl;
    first_time_ = false;
  }

  // Read Point Cloud
  try
  {
    projector_.transformLaserScanToPointCloud("/base_link",*scan_in, cloud_,listener_);
    pcl_conversions::toPCL(cloud_, cloud2_pcl);
    pcl::fromPCLPointCloud2(cloud2_pcl, cloud_pcl);

  }
  catch (tf::TransformException& e)
  {
    std::cout << e.what() << std::endl;
    return;
  }

  // Transform Image
  try
  {
    cv_ptr = cv_bridge::toCvCopy(img_in, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  cv_bridge::CvImage pub_img = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv_ptr->image);
  image_pub_.publish(pub_img.toImageMsg());

  LaserImageProjection::laserProjection(cloud_pcl, imagePoints);

  // Project pointcloud into image
  ProjectImage( imagePoints, cinfo_in, cloud_pcl, cv_ptr->image);

  // Draw bounding boxes on image
  drawBoundingBox(cv_ptr->image, box_list_);

  // Update times
  current_time_ = ros::Time::now();
  ros::Duration diff = current_time_ - past_time_;

  // Draw fps
  // std::string s = boost::lexical_cast<std::string>(1.0/diff.toSec()).substr(0, 4) + " fps" +
  //                 "   detect rate: " +  boost::lexical_cast<std::string>(1.0/box_duration_.toSec()).substr(0, 4);
  // cv::putText(cv_ptr->image, s, cv::Point(20, 40),
  //             cv::FONT_HERSHEY_SIMPLEX, 1,cv::Scalar(0, 255, 255), 3, 8);

  past_time_ = current_time_;

  if (pub_result_)
  {
    result_pub_.publish(cv_ptr->toImageMsg());
  }

  if (display_)
  {
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);
  }
}

void updateBox(const eyecu_msgs::boundingBoxArray::ConstPtr& msg)
{
  DepthBox box;
  box_list_.clear();
  int size = msg->boundingBoxes.size();
  //std::cout << size << std::endl;
  for (int i = 0; i < size; i++)
  {
    box.rect = cv::Rect(cv::Point(msg->boundingBoxes[i].pointA.x, msg->boundingBoxes[i].pointA.y),
                        cv::Point(msg->boundingBoxes[i].pointB.x, msg->boundingBoxes[i].pointB.y));
    box.dis = -1;
    box.name = msg->boundingBoxes[i].name;
    //std::cout << box_rect << std::endl;
    box_list_.push_back(box);
  }
  box_current_time_ = ros::Time::now();
  box_duration_ = box_current_time_ - box_past_time_;
  box_past_time_ = box_current_time_;
}

// Main function
int main(int argc, char **argv)
{
  // Set windows properties if image is displayed using OpenCV's imshow
  if (display_)
  {
    cv::namedWindow(OPENCV_WINDOW, cv::WND_PROP_FULLSCREEN);
    cv::setWindowProperty(OPENCV_WINDOW, CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
    cv::resizeWindow(OPENCV_WINDOW, 1920, 1080);
  }

  // Initalize node
  ros::init(argc, argv, "laser_image_projection");
  ros::NodeHandle n;

  // Subscribe to bounding box topic
  ros::Subscriber sub_box_ = n.subscribe("/bounding_boxes", 1, updateBox);

  // Create object
  LaserImageProjection lstopc(n);

  // Node is spinned to continue subscribing
  ros::spin();
  return 0;
}
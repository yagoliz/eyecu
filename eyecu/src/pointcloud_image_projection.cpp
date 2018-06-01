#include <laser_projection/pointcloud_image_projection.h>

PointcloudImageProjection::PointcloudImageProjection(ros::NodeHandle n):
  n_(n),
  pointcloud_sub_(n_, "/velodyne_points", 10),
  image_sub_(n_, "/front_camera/image_raw", 10),
  cinfo_sub_(n_, "/front_camera/camera_info", 10)
{
  // First we load the private parameters
  ros::NodeHandle nh_private("~");
  nh_private.param<bool>("pub_marks", pub_marks_, true);
  nh_private.param<std::string>("face_distance_topic", face_distance_topic_,"/face_distance");

  // We initialize the camera matrix
  initMatrix();

  // Show findings on RVIZ
  if (true)
  {
    marker_pub = n.advertise<visualization_msgs::Marker>("/object_show", 1);
  }

  face_distance_pub_ = n.advertise<eyecu_msgs::DistanceCamera>(face_distance_topic_, 1);

  first_time_ = true;
  sync_ = new message_filters::Synchronizer<SyncPolicy>
    ( SyncPolicy(10), pointcloud_sub_, image_sub_, cinfo_sub_);

  sync_->registerCallback(std::tr1::bind(&PointcloudImageProjection::callbackMethod,
                          this, std::tr1::placeholders::_1, std::tr1::placeholders::_2,
                          std::tr1::placeholders::_3 ));
}

void PointcloudImageProjection::initMatrix()
{
  camProjection_matrix_.resize(3, 4);
  camProjection_matrix_.setZero();
}

void PointcloudImageProjection::setTransformMatrix(const sensor_msgs::CameraInfoConstPtr& cinfo_in, tf::StampedTransform c_tf)
{
  // Declare the matrices for the Camera Projection Matrix
  MatrixXd projection_matrix;
  MatrixXd camTranslate_matrix;
  MatrixXd camRotate_matrix;

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
      projection_matrix(i, j) = cinfo_in -> P[4*i+j];
    }
  }

  // Variables to store the Euler angles of the rotation matrix
  double roll, pitch, yaw;
  tf::Matrix3x3 m(c_tf.getRotation());
  m.getRPY(roll, pitch, yaw);

  camTranslate_matrix(0, 3) =  c_tf.getOrigin().y();
  camTranslate_matrix(1, 3) =  c_tf.getOrigin().z();
  camTranslate_matrix(2, 3) = -c_tf.getOrigin().x();
  camRotate_matrix(0, 0) =  cos(yaw);
  camRotate_matrix(0, 2) =  sin(yaw);
  camRotate_matrix(2, 0) = -sin(yaw);
  camRotate_matrix(2, 2) =  cos(yaw);

  camProjection_matrix_ = projection_matrix * camTranslate_matrix * camRotate_matrix;
}

void PointcloudImageProjection::laserProjection( pcl::PointCloud<pcl::PointXYZ> &cloud,
                      MatrixXd &imagePoints)
{
  // This function will project the pointcloud to the image

  int n = cloud.width * cloud.height;
  int count = 0;

  // We resize the image points to a 4 x n matrix
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

void PointcloudImageProjection::putBoxDistance(int &x, int &y, double &distance, pcl::PointXYZ &point, std::vector<DepthBox> &list)
{
  for(int i = 0; i < list.size(); i++)
  {
    if( x > list[i].rect.x + list[i].rect.width/4
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

void PointcloudImageProjection::ProjectImage(MatrixXd &imagePoints,
                      const sensor_msgs::CameraInfoConstPtr& cinfo_in,
                      pcl::PointCloud<pcl::PointXYZ> &cloud,
                      cv::Mat &img)
{
  int x, y, d;
  int r, g, b;
  int x_shift, y_shift;
  float w;
  for (int i = 0; i < imagePoints.cols(); i++)
  {
    w = imagePoints(2, i);
    if (w > 0)
    {
      x = (int)(imagePoints(0, i) / w);
      y = (int)(imagePoints(1, i) / w);
      if (x >= 0 && x < cinfo_in->width && y >= 0 && y < cinfo_in->height)
      {
        x_shift = x;
        y_shift = y;
      }
      double distance = sqrt(pow(cloud.points[i].x, 2) + pow(cloud.points[i].y, 2));
      putBoxDistance(x_shift , y_shift, distance, cloud.points[i], box_list_);
      cv::circle( img, cv::Point(x_shift, y_shift), 3, cv::Scalar(r, g, b), -1, 8 );
    }
  }
  face_distance_pub_.publish(face_distance_);
}

void PointcloudImageProjection::putBoxOnRVIZ(int id, pcl::PointXYZ &point, std::string &str)
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
  if(str.compare("face")==0)
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

void PointcloudImageProjection::drawBoundingBox(std::vector<DepthBox> &list)
{
  for (int i = 0; i < list.size(); i++)
  {
    if(list[i].dis != -1 )
    {
      if(pub_marks_)
      {
        putBoxOnRVIZ(i, list[i].point, list[i].name);
      }
    }
  }
}

void PointcloudImageProjection::callbackMethod(const sensor_msgs::PointCloud2ConstPtr& cloud_in,
                      const sensor_msgs::ImageConstPtr& img_in,
                      const sensor_msgs::CameraInfoConstPtr& cinfo_in )
{
  // Callback functional
  // First the pointcloud objects from pcl library
  PCLPointCloud cloud_pcl_prev, cloud_pcl;

  // Now the CV Bridge pointer
  cv_bridge::CvImagePtr cv_ptr;

  // Matrix to store the projections of the PCL
  MatrixXd imagePoints;



  // First loop the transform is set
  if (first_time_)
  {
    tf::TransformListener tf_listener;
    tf::StampedTransform c_tf;
    ros::Time now = ros::Time::now();
    try
    {
      tf_listener.waitForTransform("/base_link", "/camera_link",
                                    now, ros::Duration(4.0));
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

  // Transform PointCloud2 ROS to PCL
  try
  {
    pcl::fromROSMsg(*cloud_in, cloud_pcl_prev);
    pcl_ros::transformPointCloud("/camera_link", cloud_pcl_prev, cloud_pcl, listener_);
  }
  catch (tf::TransformException& e)
  {
    std::cout << e.what() << std::endl;
    return;
  }

  // Transform from ROS Image to OpenCV Mat
  try
  {
    cv_ptr = cv_bridge::toCvCopy(img_in, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // Transform the pointcloud to image plane
  PointcloudImageProjection::laserProjection(cloud_pcl, imagePoints);

  // Project points into image
  ProjectImage(imagePoints, cinfo_in, cloud_pcl, cv_ptr->image);

  // Draw all the faces o RVIZ
  drawBoundingBox(box_list_);

  // Update times
  current_time_ = ros::Time::now();
  ros::Duration diff = current_time_ - past_time_;

  past_time_ = current_time_;
}

// Function not belonging to the class defined above
void updateBox(const eyecu_msgs::boundingBoxArray::ConstPtr& msg)
{
  DepthBox box;
  box_list_.clear();
  int size = msg->boundingBoxes.size();

  for (int i = 0; i < size; i++)
  {
    box.rect = cv::Rect(cv::Point(msg->boundingBoxes[i].pointA.x, msg->boundingBoxes[i].pointA.y),
                        cv::Point(msg->boundingBoxes[i].pointB.x, msg->boundingBoxes[i].pointB.y));
    box.dis = -1;
    box.name = msg->boundingBoxes[i].name;
    box_list_.push_back(box);
  }
  box_current_time_ = ros::Time::now();
  box_duration_ = box_current_time_ - box_past_time_;
  box_past_time_ = box_current_time_;
}

// Main function
int main(int argc, char **argv)
{
  // Initalize node
  ros::init(argc, argv, "laser_image_projection");
  ros::NodeHandle n;

  // Subscribe to bounding box topic
  ros::Subscriber sub_box_ = n.subscribe("/bounding_boxes", 1, updateBox);

  // Create object
  PointcloudImageProjection obj(n);

  // Node is spinned to continue subscribing
  ros::spin();
  return 0;
}

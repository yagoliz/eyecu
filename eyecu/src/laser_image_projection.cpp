#include <laser_projection/laser_image_projection.h>

LaserImageProjection::LaserImageProjection(ros::NodeHandle n):
  n_(n),
  laser_sub_(n_, "scan", 10),
  image_sub_(n_, "/image_raw_color", 10),
  cinfo_sub_(n_, "/camera_info", 10)
{
  ros::NodeHandle nh_private("~");
  nh_private.param<bool>("display", display_, false);
  nh_private.param<bool>("pub_marks", pub_marks_, false);
  nh_private.param<bool>("pub_theta", pub_theta_, false);
  nh_private.param<bool>("pub_result", pub_result_, false);
  //n.param<bool>("display", display_, false);
  initMatrix();
  image_transport::ImageTransport it(n);
  if (pub_marks_)
  {
    marker_pub = n.advertise<visualization_msgs::Marker>("object_show", 1);
  }
  if (pub_theta_)
  {
    theta_pub_ = n.advertise<cam_ring_msg::ThetaArray>("theta", 1);
  }
  if (pub_result_)
  {
    result_pub_ = it.advertise("result_image", 1);
  }
  image_pub_ = it.advertise("pub_image", 1);
  first_time_ = true;
  sync_ = new message_filters::Synchronizer<AppxiSyncPolicy>
    ( AppxiSyncPolicy(10), laser_sub_, image_sub_, cinfo_sub_);

  sync_->registerCallback(std::tr1::bind(&LaserImageProjection::callbackMethod,
                          this, std::tr1::placeholders::_1, std::tr1::placeholders::_2,
                          std::tr1::placeholders::_3 ));
}

void LaserImageProjection::initMatrix()
{
  camProjection_matrix_.resize(3, 4);
  camProjection_matrix_.setZero();
}

void LaserImageProjection::setTransformMatrix(const sensor_msgs::CameraInfoConstPtr& cinfo_in, tf::StampedTransform c_tf)
{
  MatrixXd projection_matrix;
  MatrixXd camTranslate_matrix;
  MatrixXd camRotate_matrix;
  double roll, pitch, yaw;

  projection_matrix.resize(3, 4);
  projection_matrix.setZero();
  camTranslate_matrix.resize(4, 4);
  camTranslate_matrix.setIdentity();
  camRotate_matrix.resize(4, 4);
  camRotate_matrix.setIdentity();

  tf::Matrix3x3 m(c_tf.getRotation());
  m.getRPY(roll, pitch, yaw);
  for(int i = 0; i < 3; i++)
  {
    for(int j = 0; j < 4; j++)
    {
      projection_matrix(i, j) = cinfo_in->P[4*i+j];
    }
  }
  camTranslate_matrix(0, 3) =  c_tf.getOrigin().y();
  camTranslate_matrix(1, 3) =  c_tf.getOrigin().z();
  camTranslate_matrix(2, 3) = -c_tf.getOrigin().x();
  camRotate_matrix(0, 0) =  cos(yaw);
  camRotate_matrix(0, 2) =  sin(yaw);
  camRotate_matrix(2, 0) = -sin(yaw);
  camRotate_matrix(2, 2) =  cos(yaw);

  camProjection_matrix_ = projection_matrix * camTranslate_matrix * camRotate_matrix;

  // std::cout << "get rotation: " << std::endl << roll << pi <<std::endl;
  // std::cout << "roste_matrix: " << std::endl <<camRotate_matrix_[num] <<std::endl;
  // std::cout << "projection_matrix: " << std::endl <<projection_matrix_[num] <<std::endl;
  // std::cout << "camTranslate_matrix: " << std::endl << camTranslate_matrix_[num] <<std::endl;
}

void LaserImageProjection::laserProjection( pcl::PointCloud<pcl::PointXYZ> &cloud,
                      MatrixXd &imagePoints)
{
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
  // std::cout << imagePoints[2].transpose() <<std::endl;
}

void LaserImageProjection::calHeatMap(float value, float min, float max, int &r, int &g, int &b )
{
  float ratio = 2.0 * (float)(value-min) / (float)(max - min);
  b = (int) 255*(1 - ratio);
  if (b < 0) b = 0;
  r = (int) 255*(ratio - 1);
  if (r < 0) r = 0;
  g = 255 - b - r;
}

void LaserImageProjection::putBoxDistance(int &x, int &y, double &distance, pcl::PointXYZ &point, std::vector<DepthBox> &list)
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
        //std::cout << list[i].point <<std::endl;
      }
      else
      {
        // // mean of points
        // list[i].dis = (list[i].dis + distance)/2;
        // list[i].point.x = (list[i].point.x + point.x)/2;
        // list[i].point.y = (list[i].point.y + point.y)/2;
        // list[i].point.z = (list[i].point.z + point.z)/2;

        // nearest point
        if(distance < list[i].dis) list[i].dis = distance;
        if(point.x < list[i].point.x) list[i].point.x = point.x;
        list[i].point.y = (list[i].point.y + point.y)/2;
        list[i].point.z = (list[i].point.z + point.z)/2;

      }
    }
  }
}

void LaserImageProjection::putProjectImage(MatrixXd &imagePoints,
                      const sensor_msgs::CameraInfoConstPtr& cinfo_in,
                      pcl::PointCloud<pcl::PointXYZ> &cloud,
                      cv::Mat &img)
{
  int x, y, d;
  int r, g, b;
  int x_shift, y_shift;
  float w;
  for (int i = 0; i < imagePoints.cols(); i++){
    w = imagePoints(2, i);
    if (w > 0)
    {
      x = (int)(imagePoints(0, i) / w);
      y = (int)(imagePoints(1, i) / w);
      if (x >= 0 && x < cinfo_in->width && y >= 0 && y < cinfo_in->height)
      {
        switch (0)
        {
          case 0:
            x_shift = x;
            y_shift = y;
            break;
          case 1:
            x_shift = x + cinfo_in->width;
            y_shift = y;
            break;
          case 2:
            x_shift = x;
            y_shift = y + cinfo_in->height;
            break;
          case 3:
            x_shift = x + cinfo_in->width;
            y_shift = y + cinfo_in->height;
            break;
          default:
            std::cout << "ERROR NUM" << std::endl;
            break;
        }
        double distance = sqrt(pow(cloud.points[i].x, 2) + pow(cloud.points[i].y, 2));
        putBoxDistance(x_shift , y_shift, distance, cloud.points[i], box_list_);
        calHeatMap(distance, HEAT_MIN, HEAT_MAX, r, g, b);
        cv::circle( img, cv::Point(x_shift, y_shift), 3, cv::Scalar(r, g, b), -1, 8 );
      }
    }
  }
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

void LaserImageProjection::drawBoundungBox(cv::Mat& img, std::vector<DepthBox> &list)
{
  cam_ring_msg::Theta theta;
  cam_ring_msg::ThetaArray theta_array;
  for (int i = 0; i < list.size(); i++)
  {
    //std::cout << list[i].rect <<std::endl;
    if (display_ || pub_result_)
    {
      cv::rectangle(img, list[i].rect, cv::Scalar(255, 255, 0), 2, 4, 0);
      cv::putText(img, list[i].name, cv::Point(list[i].rect.x, list[i].rect.y - 5),
                  cv::FONT_HERSHEY_SIMPLEX, 1,cv::Scalar(100, 255, 0), 3, 8);
    }
    if(list[i].dis != -1 )
    {
      if (display_ || pub_result_)
      {
        std::string s = boost::lexical_cast<std::string>(list[i].dis).substr(0, 5) + " m";
        cv::putText(img, s, cv::Point(list[i].rect.x + 5, list[i].rect.y + 30),
                    cv::FONT_HERSHEY_SIMPLEX, 1,cv::Scalar(0, 0, 255), 3, 8);
      }
      //std::cout << list[i].point <<std::endl;
      if(pub_marks_)
      {
        putBoxOnRVIZ(i, list[i].point, list[i].name);
      }
      if(pub_theta_)
      {
        theta.theta = atan(list[i].point.y / list[i].point.x) * 180 / PI;
        theta.dis = list[i].dis;
        theta_array.Thetas.push_back(theta);
      }
    }
  }
  if(pub_theta_)
  {
    theta_pub_.publish(theta_array);
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

  putProjectImage( imagePoints, cinfo_in, cloud_pcl, cv_ptr->image);
  cv::Mat combine_img;
  combine_img =  cv::Mat::zeros(cinfo_in->height, cinfo_in->width, CV_8UC3);
  cv_ptr->image.copyTo(combine_img(cv::Rect(0, 0, cv_ptr->image.cols, cv_ptr->image.rows)));
  drawBoundungBox(combine_img, box_list_);
  current_time_ = ros::Time::now();
  ros::Duration diff = current_time_ - past_time_;
  std::string s = boost::lexical_cast<std::string>(1.0/diff.toSec()).substr(0, 4) + " fps" +
                  "   detect rate: " +  boost::lexical_cast<std::string>(1.0/box_duration_.toSec()).substr(0, 4);
  cv::putText(combine_img, s, cv::Point(20, 40),
              cv::FONT_HERSHEY_SIMPLEX, 1,cv::Scalar(0, 255, 255), 3, 8);
  // std::cout << "fps: " << 1.0 / diff.toSec() << std::endl;
  past_time_ = current_time_;
  //image_pub_.publish(pub_img.toImageMsg());
  if (pub_result_)
  {
    cv_bridge::CvImage pub_result_img = cv_bridge::CvImage(std_msgs::Header(), "bgr8", combine_img);
    result_pub_.publish(pub_result_img.toImageMsg());
  }

  if (display_)
  {
    cv::imshow(OPENCV_WINDOW, combine_img);
    cv::waitKey(3);
  }
}

void updateBox(const deep_learning_msgs::boundingBoxArray::ConstPtr& msg)
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

int main(int argc, char **argv)
{
  if (display_)
  {
    cv::namedWindow(OPENCV_WINDOW, cv::WND_PROP_FULLSCREEN);
    cv::setWindowProperty(OPENCV_WINDOW, CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
    //cv::namedWindow(OPENCV_WINDOW, cv::WINDOW_NORMAL);
    cv::resizeWindow(OPENCV_WINDOW, 1920, 1080);
  }
  ros::init(argc, argv, "laser_image_projection");
  ros::NodeHandle n;
  ros::Subscriber sub_box_ = n.subscribe("/deep_boundingBox", 1, updateBox);
  LaserImageProjection lstopc(n);
  ros::spin();
  return 0;
}

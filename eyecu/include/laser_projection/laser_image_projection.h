// LaserScan projection node
// For any questions contact: yagol@mit.edu

// Standard libraries
#include <string>
#include <tr1/functional>

// Non-standard libraries
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include "pcl_ros/transforms.h"
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

#include <Eigen/Dense>

// ROS libraries
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <laser_geometry/laser_geometry.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

// ROS messages
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/String.h>
#include <sensor_msgs/image_encodings.h>
#include <visualization_msgs/Marker.h>

// Custom ROS msgs
#include <eyecu_msgs/boundingBoxArray.h>
#include <eyecu_msgs/boundingBox.h>
#include <eyecu_msgs/DistanceCamera.h>

#define HEAT_MAX 20
#define HEAT_MIN 0

#define PI 3.14159265

using namespace Eigen;
using namespace sensor_msgs;

// This type definition will be used to synchronize data coming from velodyne
// and camera
typedef message_filters::sync_policies::ApproximateTime
  < sensor_msgs::LaserScan,
    sensor_msgs::Image,
    sensor_msgs::CameraInfo> SyncPolicy;

// This type will be used to process the pointcloud
typedef pcl::PointCloud<pcl::PointXYZ> PCLPointCloud;

// This struct will store the bounding boxes that come from the OpenCV, Dlib or
// Tensorflow nodes
struct DepthBox{
  std::string name;
  cv::Rect rect;
  float dis;
  pcl::PointXYZ point;
};

std::vector<DepthBox> box_list_;

// Aditional variables
static const std::string OPENCV_WINDOW = "Webcam";

ros::Time box_past_time_, box_current_time_;
ros::Duration box_duration_;

bool display_;

class LaserImageProjection
{
  public:

      // ROS nodehandle, listener, publishers and subscribers
      ros::NodeHandle n_;
      laser_geometry::LaserProjection projector_;
      tf::TransformListener listener_;
      ros::Publisher marker_pub, face_distance_pub_;
      image_transport::Publisher image_pub_, result_pub_;

      // Message synchronizing variables
      message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_;
      message_filters::Subscriber<sensor_msgs::Image> image_sub_;
      message_filters::Subscriber<sensor_msgs::CameraInfo> cinfo_sub_;

      message_filters::Synchronizer<SyncPolicy> *sync_;

      // ROS variables
      visualization_msgs::Marker marker;
      sensor_msgs::PointCloud2 cloud_;
      eyecu_msgs::DistanceCamera face_distance_;
      ros::Time past_time_, current_time_;

      // Eigen variables
      MatrixXd camProjection_matrix_;

      // ROS params
      bool first_time_, pub_marks_, pub_result_;

      // Member function definitions
      LaserImageProjection(ros::NodeHandle n);

      void initMatrix();

      void setTransformMatrix(const sensor_msgs::CameraInfoConstPtr& cinfo_in,
                              tf::StampedTransform c_tf);


      void laserProjection(pcl::PointCloud<pcl::PointXYZ> &cloud,
                            MatrixXd &imagePoints);

      void callHeatMap(float value,
                       float min,
                       float max,
                       int &r,
                       int &g,
                       int &b );

      void putBoxDistance(int &x,
                           int &y,
                           double &distance,
                           pcl::PointXYZ &point,
                           std::vector<DepthBox> &list);

      void ProjectImage(MatrixXd &imagePoints,
                           const sensor_msgs::CameraInfoConstPtr& cinfo_in,
                           pcl::PointCloud<pcl::PointXYZ> &cloud,
                           cv::Mat &img);

      void putBoxOnRVIZ(int id, pcl::PointXYZ &point,
                        std::string &str);

      void drawBoundingBox(cv::Mat& img,
                            std::vector<DepthBox> &list);

      void callbackMethod (const sensor_msgs::LaserScanConstPtr& scan_in,
                            const sensor_msgs::ImageConstPtr& img_in,
                            const sensor_msgs::CameraInfoConstPtr& cinfo_in );

};

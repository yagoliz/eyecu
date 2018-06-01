// LaserScan projection node
// For any questions contact: yagol@mit.edu

// Standard libraries

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/String.h>
#include <laser_geometry/laser_geometry.h>
#include <visualization_msgs/Marker.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

#include <Eigen/Dense>

#include <eyecu_msgs/boundingBoxArray.h>
#include <eyecu_msgs/boundingBox.h>

#include <string>

#include <tr1/functional>

#define HEAT_MAX 20
#define HEAT_MIN 0

#define PI 3.14159265

using namespace Eigen;
using namespace sensor_msgs;

typedef message_filters::sync_policies::ApproximateTime
  < sensor_msgs::LaserScan,
    sensor_msgs::Image,
    sensor_msgs::CameraInfo> AppxiSyncPolicy;

typedef pcl::PointCloud<pcl::PointXYZ> PCLPointCloud;

struct DepthBox{
  std::string name;
  cv::Rect rect;
  float dis;
  pcl::PointXYZ point;
};

static const std::string OPENCV_WINDOW = "Webcam";

std::vector<DepthBox> box_list_;

ros::Time box_past_time_, box_current_time_;
ros::Duration box_duration_;

bool display_;

class LaserImageProjection
{
  public:

      ros::NodeHandle n_;
      laser_geometry::LaserProjection projector_;
      tf::TransformListener listener_;
      ros::Publisher marker_pub, theta_pub_;
      image_transport::Publisher image_pub_, result_pub_;

      visualization_msgs::Marker marker;

      message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_;
      message_filters::Subscriber<sensor_msgs::Image> image_sub_;
      message_filters::Subscriber<sensor_msgs::CameraInfo> cinfo_sub_;

      message_filters::Synchronizer<AppxiSyncPolicy> *sync_;

      sensor_msgs::PointCloud2 cloud_;

      MatrixXd camProjection_matrix_;

      bool first_time_, pub_marks_, pub_theta_, pub_result_;

      ros::Time past_time_, current_time_;

      LaserImageProjection(ros::NodeHandle n);

      void initMatrix();

      void setTransformMatrix(const sensor_msgs::CameraInfoConstPtr& cinfo_in,
                         tf::StampedTransform c_tf);


      void laserProjection(pcl::PointCloud<pcl::PointXYZ> &cloud,
                      MatrixXd &imagePoints);

      void calHeatMap(float value,
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

      void putProjectImage(MatrixXd &imagePoints,
                     const sensor_msgs::CameraInfoConstPtr& cinfo_in,
                     pcl::PointCloud<pcl::PointXYZ> &cloud,
                     cv::Mat &img);

      void putBoxOnRVIZ(int id, pcl::PointXYZ &point,
                   std::string &str);

      void drawBoundungBox(cv::Mat& img,
                      std::vector<DepthBox> &list);

      void callbackMethod (const sensor_msgs::LaserScanConstPtr& scan_in,
                      const sensor_msgs::ImageConstPtr& img_in,
                      const sensor_msgs::CameraInfoConstPtr& cinfo_in );

};

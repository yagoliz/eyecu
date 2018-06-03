// Convert apriltag msg to face distance msg
// For any questions contact: yagol@mit.edu

// Standard libraries
#include <string>

// ROS libraries
#include <ros/ros.h>

// Custom ROS messages
#include <eyecu_msgs/DistanceCamera.h>
#include <apriltags_ros/AprilTagDetection.h>
#include <apriltags_ros/AprilTagDetectionArray.h>

// Global variable for face distance topic
ros::Publisher face_pub;
eyecu_msgs::DistanceCamera face_distance;

// Subscriber function
void callback(const apriltags_ros::AprilTagDetectionArrayPtr& tags)
{
  // We first get the closes tag
  float dist = 100.0;
  int closest = -1;
  for (int i=0; i<tags->detections.size(); i++)
  {
    if (tags->detections[i].pose.pose.position.z < dist)
    {
      closest = i;
    }
  }

  // In case there is an error when detecting tags
  if (closest >= 0)
  {
    face_distance.X = tags->detections[closest].pose.pose.position.x;
    face_distance.Y = tags->detections[closest].pose.pose.position.y;
    face_distance.Z = tags->detections[closest].pose.pose.position.z;

    // Publish the face distance
    face_pub.publish(face_distance);
  }
}

// Main function
int main(int argc, char** argv)
{
  // Initialize node
  ros::init(argc, argv, "apriltag_to_face_distance");
  ros::NodeHandle nh;

  // Subscribe to apriltag topic and avertise publisher
  ros::Subscriber sub_tag = nh.subscribe("/tag_detections", 1, callback);
  face_pub = nh.advertise<eyecu_msgs::DistanceCamera>("/face_distance", 1);

  // Spin the node until killed
  ros::spin();
  return 0;
}

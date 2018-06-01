/* Code developed by Yago Lizarribar at MIT Media Lab based on Lentin Joseph's
 * face tracker control
 *
 * This code has been modified to move 2 servos at the same time
 * For any questions email: yagol@mit.edu
*/

#include <eyecu_control.h>

FaceTracker::FaceTracker() {

  // Loading default values
  face_distance_topic = "/face_distance";
  pan_controller_command = "pan_controller/command";
  tilt_controller_command = "tilt_controller/command";
  target_frame = "right_eye_link";
  source_frame = "camera_link";
  servomaxx = 1.0;
  servomaxy = 1.0;
  debug = false;

  try
  {
    nh.getParam("face_distance_topic" , face_distance_topic );
    nh.getParam("pan_controller_command" , pan_controller_command );
    nh.getParam("tilt_controller_command", tilt_controller_command);
    nh.getParam("servomaxx", servomaxx);
    nh.getParam("servomaxy", servomaxy);
    nh.getParam("debug", debug);
    nh.getParam("target_frame", target_frame);
    nh.getParam("source_frame", source_frame);

    ROS_INFO("Successfully Loaded tracking parameters");
  }
  catch(int e)
  {
    ROS_WARN("Parameters are not properly loaded from file, loading defaults");
  }

  // Transform variables
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  //Create subscriber and publisher objects
  face_subscriber = nh.subscribe(face_distance_topic, 10 , &FaceTracker::face_callback, this);
  pan_control  = nh.advertise<std_msgs::Float64>(pan_controller_command, 1);
  tilt_control = nh.advertise<std_msgs::Float64>(tilt_controller_command, 1);

  // Debugger publisher
  debug_pub = nh.advertise<geometry_msgs::TransformStamped>("/debug", 1);

  // Initial poses
  pan_control .publish(pan_pose);
  tilt_control.publish(tilt_pose);

  try {
    transformStamped = tfBuffer.lookupTransform(target_frame, source_frame, ros::Time(1), ros::Duration(5));
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("%s",ex.what());
    ros::Duration(1.0).sleep();
  }

}

// Callback function
void FaceTracker::face_callback(const eyecu::DistanceCamera::ConstPtr& msg) {

  if (debug) {
    ROS_INFO("Recieved X = [%f], Y = [%f], Z = [%f]", msg->X, msg->Y, msg->Z);
  }

  t_in(0) =  msg->Z/100;
  t_in(1) = -msg->X/100;
  t_in(2) = -msg->Y/100;

  // std::cout << t_in << std::endl;

  tf2::doTransform(t_in, t_out, transformStamped);

  // std::cout << t_out << std::endl;

  //Calling track face function
  track_face(t_out(0), t_out(1), t_out(2));
}

void FaceTracker::track_face(double x, double y, double z) {

  double phi   =  atan2(x,y) - PI/2;
  double theta = -atan2(z,sqrt(pow(x,2)+pow(y,2)));

  // int sign_phi   = (int) (phi  /abs  (phi));
  // int sign_theta = (int) (theta/abs(theta));
  //
  // phi   = sign_phi   * std::min((float)abs  (phi), servomaxx);
  // theta = sign_theta * std::min((float)abs(theta), servomaxy);

  pan_pose.data = phi;
  tilt_pose.data = theta;

  if (debug){
    // printf("phi:%f, theta: %f \n", phi, theta);
    debug_pub.publish(transformStamped);
  }

  pan_control .publish(pan_pose);
  tilt_control.publish(tilt_pose);

}

/******************************************************************************/
// Main function
int main(int argc, char **argv) {

  // Initializing ROS node
  ros::init(argc, argv, "eyecu_control");

  // Initialize FaceTracker object
  FaceTracker ft;

  // Spin node
  ros::spin();
  return 0;
}

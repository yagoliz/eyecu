/* Code developed by Yago Lizarribar at MIT Media Lab based on Lentin Joseph's
 * face tracker control
 *
 * This code has been modified to move 2 servos at the same time
 * For any questions email: yagol@mit.edu
*/

#include <math.h>
#include <motor_state_broadcaster.h>


MotorStateBroadcaster::MotorStateBroadcaster(ros::NodeHandle nh, ros::NodeHandle nhp) {

  pan_topic  = "pan_controller/state" ;
  tilt_topic = "tilt_controller/state";

  eye_link  = "right_eye_link";
  tilt_link = "tilt_link";
  gaze_link = "gaze_link";

  tilt_x =  0.02;
  tilt_y =  0.00;
  tilt_z = -0.04;

  gaze_x = 0.03;
  gaze_y = 0.00;
  gaze_z = 0.00;

  try {
    nhp.getParam("pan_topic" , pan_topic );
    nhp.getParam("tilt_topic", tilt_topic);
    nhp.getParam("eye_link"  , eye_link  );
    nhp.getParam("tilt_link" , tilt_link );
    nhp.getParam("gaze_link" , gaze_link );
    nhp.getParam("tilt_x" , tilt_x);
    nhp.getParam("tilt_y" , tilt_y);
    nhp.getParam("tilt_z" , tilt_z);
    nhp.getParam("gaze_x" , gaze_x);
    nhp.getParam("gaze_y" , gaze_y);
    nhp.getParam("gaze_z" , gaze_z);
  }
  catch (int e) {
    ROS_WARN("Parameters are not properly loaded from file, loading defaults");
  }

  pan_subscriber.subscribe (nh, pan_topic , 1);
  tilt_subscriber.subscribe(nh, tilt_topic, 1);

  sync = new message_filters::Synchronizer<SyncPolicy> (SyncPolicy(10), pan_subscriber, tilt_subscriber);
  sync->registerCallback(boost::bind(&MotorStateBroadcaster::callback, this, _1, _2));

}

void MotorStateBroadcaster::callback(const dynamixel_msgs::JointStateConstPtr& pan, const dynamixel_msgs::JointStateConstPtr& tilt) {

  static tf2_ros::TransformBroadcaster broadcaster;

  float pan_angle  = pan ->current_pos;
  float tilt_angle = tilt->current_pos;

  // Transform with pan angle
  transform_pan.header.stamp  = ros::Time::now();
  transform_pan.header.frame_id = eye_link ;
  transform_pan.child_frame_id  = tilt_link;

  transform_pan.transform.translation.x =  tilt_x*cos(pan_angle);
  transform_pan.transform.translation.y =  tilt_y*sin(pan_angle);
  transform_pan.transform.translation.z = -tilt_z               ;

  tf2::Quaternion q;
  q.setRPY(0, 0, pan_angle);
  transform_pan.transform.rotation.x = q.x();
  transform_pan.transform.rotation.y = q.y();
  transform_pan.transform.rotation.z = q.z();
  transform_pan.transform.rotation.w = q.w();

  broadcaster.sendTransform(transform_pan);

  // Transform with tilt angle
  transform_tilt.header.stamp  = ros::Time::now();
  transform_tilt.header.frame_id = tilt_link;
  transform_tilt.child_frame_id  = gaze_link;

  transform_tilt.transform.translation.x = tilt_x*cos(tilt_angle);
  transform_tilt.transform.translation.y = tilt_y                ;
  transform_tilt.transform.translation.z = tilt_z*sin(tilt_angle);

  q.setRPY(0, tilt_angle, 0);
  transform_tilt.transform.rotation.x = q.x();
  transform_tilt.transform.rotation.y = q.y();
  transform_tilt.transform.rotation.z = q.z();
  transform_tilt.transform.rotation.w = q.w();

  broadcaster.sendTransform(transform_tilt);

}


int main(int argc, char **argv) {

  // Initializing ROS node
  ros::init(argc, argv, "motor_state_broadcaster");

  // Initialize Motor broadcaster object
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");
  MotorStateBroadcaster msc(nh, nhp);

  //  Spin node
  ros::spin();
  return 0;

}

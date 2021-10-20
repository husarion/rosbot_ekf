#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include <string>

#define ODOM_COV 0.005
#define MAIN_LOOP_RATE 20

ros::Publisher *odom_pub_ptr;
std::string tf_prefix_;
bool has_prefix;

void poseCallback(const geometry_msgs::PoseStamped &pose_msg)
{
  nav_msgs::Odometry odom_msg;

  odom_msg.pose.pose.orientation.x = pose_msg.pose.orientation.x;
  odom_msg.pose.pose.orientation.y = pose_msg.pose.orientation.y;
  odom_msg.pose.pose.orientation.z = pose_msg.pose.orientation.z;
  odom_msg.pose.pose.orientation.w = pose_msg.pose.orientation.w;

  odom_msg.pose.pose.position.x = pose_msg.pose.position.x;
  odom_msg.pose.pose.position.y = pose_msg.pose.position.y;
  odom_msg.pose.pose.position.z = pose_msg.pose.position.z;

  odom_msg.pose.covariance[0] = ODOM_COV;
  odom_msg.pose.covariance[7] = ODOM_COV;
  odom_msg.pose.covariance[14] = ODOM_COV;
  odom_msg.pose.covariance[21] = ODOM_COV;
  odom_msg.pose.covariance[28] = ODOM_COV;
  odom_msg.pose.covariance[35] = ODOM_COV;

  odom_msg.header = pose_msg.header;
  if (has_prefix)
  {
    odom_msg.header.frame_id = tf_prefix_ + "/odom";
  }
  else
  {
    odom_msg.header.frame_id = "odom";
  }
  odom_pub_ptr->publish(odom_msg);
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "msgs_conversion");

  ros::NodeHandle n;

  has_prefix = ros::param::get("~tf_prefix", tf_prefix_);

  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom/wheel", 1);
  odom_pub_ptr = &odom_pub;

  ros::Subscriber pose_sub = n.subscribe("pose", 1000, poseCallback);

  ros::Rate loop_rate(MAIN_LOOP_RATE);

  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

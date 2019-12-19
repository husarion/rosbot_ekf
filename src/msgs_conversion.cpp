#include "ros/ros.h"
#include "rosbot_ekf/Imu.h"       
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"

#define ODOM_COV 0.005
#define IMU_ORIENTATION_COV 0.05
#define IMU_ANG_VEL_COV 0.1
#define IMU_LIN_ACCEL_COV 0.5
#define PI 3.1415
#define G_ACCEL 9.8066

ros::Publisher *odom_pub_ptr;
ros::Publisher *imu_pub_ptr;
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
  if(has_prefix){
  odom_msg.header.frame_id = tf_prefix_ + "/odom";
  }
  else{
    odom_msg.header.frame_id = "odom";
  }
  odom_pub_ptr->publish(odom_msg);

}

void mpuCallback(const rosbot_ekf::Imu &mpu_msg)
{
  sensor_msgs::Imu imu_msg;
  
  imu_msg.header = mpu_msg.header;
  if(has_prefix){
  imu_msg.header.frame_id = tf_prefix_ + "/imu_link";
}
else{
  imu_msg.header.frame_id = "imu_link";
}
  imu_msg.orientation.x = mpu_msg.orientation.x;
  imu_msg.orientation.y = mpu_msg.orientation.y;
  imu_msg.orientation.z = mpu_msg.orientation.z;
  imu_msg.orientation.w = mpu_msg.orientation.w;
  
  imu_msg.angular_velocity.x = mpu_msg.angular_velocity[0] * PI / 180;
  imu_msg.angular_velocity.y = mpu_msg.angular_velocity[1] * PI / 180;
  imu_msg.angular_velocity.z = mpu_msg.angular_velocity[2] * PI / 180;

  imu_msg.linear_acceleration.x = mpu_msg.linear_acceleration[0] * G_ACCEL;
  imu_msg.linear_acceleration.y = mpu_msg.linear_acceleration[1] * G_ACCEL; 
  imu_msg.linear_acceleration.z = mpu_msg.linear_acceleration[2] * G_ACCEL; 

  imu_msg.orientation_covariance[0] = IMU_ORIENTATION_COV;
  imu_msg.orientation_covariance[4] = IMU_ORIENTATION_COV;
  imu_msg.orientation_covariance[8] = IMU_ORIENTATION_COV;

  imu_msg.angular_velocity_covariance[0] = IMU_ANG_VEL_COV;
  imu_msg.angular_velocity_covariance[4] = IMU_ANG_VEL_COV;
  imu_msg.angular_velocity_covariance[8] = IMU_ANG_VEL_COV;

  imu_msg.linear_acceleration_covariance[0] = IMU_LIN_ACCEL_COV;
  imu_msg.linear_acceleration_covariance[4] = IMU_LIN_ACCEL_COV;
  imu_msg.linear_acceleration_covariance[8] = IMU_LIN_ACCEL_COV;
  
  

  imu_pub_ptr->publish(imu_msg);

}


int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "msgs_conversion");


  ros::NodeHandle n;

  
  has_prefix=ros::param::get("~tf_prefix", tf_prefix_);
  ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("imu", 1);
  imu_pub_ptr = &imu_pub;
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom/wheel", 1);
  odom_pub_ptr = &odom_pub;

  ros::Subscriber mpu_sub = n.subscribe("mpu9250", 1000, mpuCallback);
  ros::Subscriber pose_sub = n.subscribe("pose", 1000, poseCallback);

  ros::Rate loop_rate(20);

  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
   
  }


  return 0;
}

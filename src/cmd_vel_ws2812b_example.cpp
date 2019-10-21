#include "ros/ros.h"
#include "rosbot_ekf/Configuration.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <cmath>

void cmdCallback(const geometry_msgs::Twist &msg)
{
    rosbot_ekf::Configuration configuration_msg;
    
    static int last_state = 0;
    // 1 - moving
    // 2 - stop

    configuration_msg.request.command = "SANI";

    if (fabs(msg.linear.x) > 0.05 || fabs(msg.angular.z) > 0.05)
    {
        if(last_state == 1)
            return;
        configuration_msg.request.data = "S #00aa00";
        ros::service::waitForService("config");
        if (ros::service::call("config", configuration_msg))
        {
            last_state = 1;
        }
    }
    else 
    {
        if(last_state == 2)
            return;
        configuration_msg.request.data = "S #aa0000";
        ros::service::waitForService("config");
        if (ros::service::call("config", configuration_msg))
        {
            last_state = 2;
        }
    }
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "cmd_vel_ws2812b_example");
    ros::NodeHandle nh;
    ros::Subscriber cmd_sub = nh.subscribe("cmd_vel", 1, cmdCallback);
    ROS_INFO("cmd_vel_ws2812b_example: subscription was made");
    ros::spin();
    return 0;
}

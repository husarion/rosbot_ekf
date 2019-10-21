#include "ros/ros.h"
#include "rosbot_ekf/Configuration.h"
#include "std_msgs/String.h"
#include <actionlib/client/terminal_state.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <actionlib_msgs/GoalStatus.h>

void statusCallback(const actionlib_msgs::GoalStatusArray &msg)
{
    static int last_status;

    if (msg.status_list.size())
    {
        rosbot_ekf::Configuration configuration_msg;
        
        int status = msg.status_list[0].status;
        if (status != last_status)
        {
            configuration_msg.request.command = "SANI";

            switch(status)
            {
                case 0: // PENDING
                    configuration_msg.request.data = "S #000000";
                    break;
                case 1: // ACTIVE
                    configuration_msg.request.data = "F #0038ff";
                    break;
                case 2: // PREEMPTED
                    configuration_msg.request.data = "F #fda600";
                    break;
                case 3: // SUCCEEDED
                    configuration_msg.request.data = "F #32ae00";
                    break;
                case 4: // ABORTED
                    configuration_msg.request.data = "B #c90000";
                    break;
                default:
                    return;
            }

            ros::service::waitForService("config");
            if (ros::service::call("config", configuration_msg))
            {
                ROS_INFO("New move_base state!");
                last_status = status;
            }
        }
    }
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "move_base_ws2812b_example");
    ros::NodeHandle nh;
    ros::Subscriber move_base_sub =
        nh.subscribe("move_base/status", 1, statusCallback);
    ROS_INFO("move_base_ws2812b_example: subscription was made");
    ros::spin();

    return 0;
}

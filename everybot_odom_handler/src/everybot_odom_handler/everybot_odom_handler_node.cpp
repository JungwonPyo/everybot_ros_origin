#include "everybot_odom_handler/everybot_odom_handler.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "everybot_odom_handler");

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    everybot::COdomHandler node(std::make_shared<ros::NodeHandle>(nh),
                                std::make_shared<ros::NodeHandle>(pnh));

    ros::MultiThreadedSpinner spinner(1); // number of thread
    spinner.spin();
    return 0;
}
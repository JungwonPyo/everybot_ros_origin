#include "odometry.h"

COdometry*  pCOdometry;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ts800_odometry_node");
    ros::NodeHandle nh;

    //direct of get wheel + odom + pixart data from the system if using thread(static)
    pCOdometry = new COdometry(&nh);
    pCOdometry->run();
    
    ros::shutdown();

    if ( pCOdometry != nullptr )
        delete pCOdometry;
 
    return 0;
}
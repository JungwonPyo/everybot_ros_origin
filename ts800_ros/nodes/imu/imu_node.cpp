#include "imu.h"

ImuNode::ImuNode()
{
  bool init_result = init();

  sleep(1);
  ROS_ASSERT(init_result);
}

ImuNode::~ImuNode()
{
}

bool ImuNode::init()
{
  publisher_  = nh_.advertise<sensor_msgs::Imu>("imu", 10);
  subscriber_ = nh_.subscribe("/imu/data_raw", 10, &ImuNode::msgCallback, this);
  return true;
}

void ImuNode::msgCallback(const sensor_msgs::ImuConstPtr imu_in)
{
  if (last_published_time_.isZero() || imu_in->header.stamp > last_published_time_)
  {
    last_published_time_ = imu_in->header.stamp;
    sensor_msgs::Imu imu_out = *imu_in;
    imu_out.linear_acceleration.x = 0.0;
    imu_out.linear_acceleration.y = 0.0;
    imu_out.linear_acceleration.z = GRAVITY;
    publisher_.publish(imu_out);
  }
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "ts800_imu_node");

  ImuNode imu_node;

  ros::spin();

  return 0;
}
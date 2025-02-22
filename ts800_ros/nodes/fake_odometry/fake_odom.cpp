#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>

int cnt_end = 0;
double vx = 0.0;
double vth = 0.0;

void odomCallback(const geometry_msgs::TwistConstPtr &cmd_vel)
{
	vx = cmd_vel->linear.x;
	vth = cmd_vel->angular.z;
}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "fake_odom");

	ros::NodeHandle n;
	ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
	ros::Publisher joint_state_pub = n.advertise<sensor_msgs::JointState>("joint_states", 50);
	ros::Subscriber sub = n.subscribe("cmd_vel", 1000, odomCallback);
	tf::TransformBroadcaster transform;

	double x = 0.0;
	double y = 0.0;
	double th = 0.0;

	ros::Time current_time, last_time;
	current_time = ros::Time::now();
	last_time = ros::Time::now();

	ros::Rate rate(10);

	while (n.ok())
	{
		// if(++cnt_end > 200){ROS_WARN("fake odom END!!");break;}
		current_time = ros::Time::now();

		double dt = (current_time - last_time).toSec();
		double delta_x = vx * cos(th) * dt;
		double delta_y = vx * sin(th) * dt;
		double delta_th = vth * dt;

		x += delta_x;
		y += delta_y;
		th += delta_th;

		// x = 0.0;
		// y = 0.0;
		// th = 0.0;

		geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

		// publish the transform odom --> base_footprint
		geometry_msgs::TransformStamped odom_trans;
		odom_trans.header.stamp = current_time;
		// odom_trans.header.seq = seq++;
		odom_trans.header.frame_id = "odom";
		odom_trans.child_frame_id = "base_link";
		// odom_trans.header.frame_id = "base_link";
		// odom_trans.child_frame_id = "odom_combined";
		odom_trans.transform.translation.x = x;
		odom_trans.transform.translation.y = y;
		odom_trans.transform.translation.z = 0.0;
		odom_trans.transform.rotation = odom_quat;
		transform.sendTransform(odom_trans);

		// publis /odom
		nav_msgs::Odometry odom;
		odom.header.stamp = current_time;
		odom.header.frame_id = "odom";
		odom.child_frame_id = "base_link";
		// odom.header.frame_id = "base_link";
		// odom.child_frame_id = "odom_combined";
		odom.pose.pose.position.x = x;
		odom.pose.pose.position.y = y;
		odom.pose.pose.position.z = 0.0;
		odom.pose.pose.orientation = odom_quat;
		odom.twist.twist.linear.x = vx;
		odom.twist.twist.angular.z = vth;
		odom_pub.publish(odom);

		// publish joint states
		// sensor_msgs::JointState joint_state;
		// joint_state.header.stamp = current_time;
		// joint_state.name.push_back("left_wheel_joint");
		// joint_state.position.push_back(0);
		// joint_state.velocity.push_back(0);
		// joint_state.effort.push_back(0);
		// joint_state.name.push_back("right_wheel_joint");
		// joint_state.position.push_back(0);
		// joint_state.velocity.push_back(0);
		// joint_state.effort.push_back(0);
		// joint_state_pub.publish(joint_state);

		last_time = current_time;
		ros::spinOnce();
		usleep(10000);
	}
}
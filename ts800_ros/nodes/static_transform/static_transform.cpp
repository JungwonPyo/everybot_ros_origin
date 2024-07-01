/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <cstdio>
#include "tf/transform_broadcaster.h"

class TransformSender_node
{
public:
  ros::NodeHandle node_;
  //constructor
  TransformSender_node(double x, double y, double z, double yaw, double pitch, double roll, ros::Time time, const std::string& frame_id, const std::string& child_frame_id)
  { 
    tf::Quaternion q;
    q.setRPY(roll, pitch,yaw);
    transform_ = tf::StampedTransform(tf::Transform(q, tf::Vector3(x,y,z)), time, frame_id, child_frame_id );
  };
  TransformSender_node(double x, double y, double z, double qx, double qy, double qz, double qw, ros::Time time, const std::string& frame_id, const std::string& child_frame_id) :
    transform_(tf::Transform(tf::Quaternion(qx,qy,qz,qw), tf::Vector3(x,y,z)), time, frame_id, child_frame_id){};
  //Clean up ros connections
  ~TransformSender_node() { }

  //A pointer to the rosTFServer class
  tf::TransformBroadcaster broadcaster;



  // A function to call to send data periodically
  void send (ros::Time time) {
    transform_.stamp_ = time;
    broadcaster.sendTransform(transform_);
  };

private:
  tf::StampedTransform transform_;

};

/**
 * @brief 
 * https://github.com/davheld/tf/blob/master/src/static_transform_publisher.cpp
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char ** argv)
{
    //Initialize ROS
    ros::init(argc, argv,"Static_Transform_Node", ros::init_options::AnonymousName);

    double x, y, z, yaw, roll, pitch, duration;
    std::string frame_id, child_frame_id;
    /*
        Run 3i LiDAR TF : TS800 WS 2-2
        #2-1샘플( -2.97 rad = -170.0 deg ) 
        #3-1샘플(-2.88 rad == -165 deg)
        lidar arg= base_link => m "x,y,z,yaw,roll,pitch"
        <node pkg="tf" type="static_transform_publisher" name="base_link_to_laser1"
            args="-0.175 0.0 0.0 -2.88 0.0 0.0 /base_link /laser 40" launch-prefix="nice -n 10"/>  
    */

    x = -0.175;
    y = 0.0;
    z = 0.0;
    yaw = -2.88;
    roll = 0.0;
    pitch = 0.0;
    duration = 40.0;
    frame_id = "/base_link";
    child_frame_id = "/laser";

    ros::Duration sleeper(duration/1000.0);
    
    TransformSender_node transform_sender(x, y, z,
                                yaw, roll, pitch,
                                ros::Time() + sleeper, //Future dating to allow slower sending w/o timeout
                                frame_id, child_frame_id);

    while(transform_sender.node_.ok())
    {
        transform_sender.send(ros::Time::now() + sleeper);
        sleeper.sleep();
    }

    return 0;
};
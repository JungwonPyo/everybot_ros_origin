#include <iostream>
#include "ros/ros.h"
#include "visualization_msgs/MarkerArray.h"

class CTrajectoryEditorNode
{
private:
    ros::NodeHandle* pNh;
    ros::Publisher trajectoryLinePub;
    ros::Publisher trajectoryCleanedAreaPub;
    ros::Subscriber trajectorySub;
    uint32_t seq;

public:
    CTrajectoryEditorNode(ros::NodeHandle* pNh)
    {
        this->pNh = pNh;
        seq = 0;

        trajectoryLinePub = pNh->advertise<visualization_msgs::MarkerArray>("trajectory/line", 5);
        trajectoryCleanedAreaPub = pNh->advertise<visualization_msgs::MarkerArray>("trajectory/cleaned_area", 5);
        trajectorySub = pNh->subscribe<visualization_msgs::MarkerArray>("/trajectory_node_list", 5, boost::bind(&CTrajectoryEditorNode::trajectoryCb, this, _1));
    }

    ~CTrajectoryEditorNode() {}

    void loop()
    {
        ros::Rate rate(5); // 200ms
        while(ros::ok())
        {
            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    void trajectoryCb(const visualization_msgs::MarkerArray::ConstPtr &msg)
    {
        visualization_msgs::MarkerArray latestTrajectory;
        filterLatestTrajectory(*msg, latestTrajectory);

        seq++;
        publishTrajectoryLine(latestTrajectory);
        publishTrajectoryCleanedArea(latestTrajectory);
    }

    void filterLatestTrajectory(const visualization_msgs::MarkerArray& input, visualization_msgs::MarkerArray& output)
    {
        int id, latestId = 0;
        for( visualization_msgs::Marker marker : input.markers)
        {
            id = std::stoi(marker.ns.substr(10));
            latestId = latestId < id ? id : latestId; // 최신 id는 가장 커야함.
        }

        for( visualization_msgs::Marker marker : input.markers )
        {
            id = std::stoi(marker.ns.substr(10));
            if(id==latestId)
            {
                output.markers.push_back(marker);
            }
        }
    }

    void publishTrajectoryLine(visualization_msgs::MarkerArray& input)
    {
        visualization_msgs::MarkerArray outputs;

        int id = 0;
        for( visualization_msgs::Marker marker : input.markers)
        {
            marker.id = id++;
            marker.header.seq = this->seq;
            marker.header.stamp = ros::Time::now();
            marker.ns = "line";
            marker.pose.position.z = 0.1;
            marker.scale.x = 0.03;
            marker.color.r = 1.0;
            marker.color.g = 1.0;
            marker.color.b = 1.0;
            marker.color.a = 0.9;
            outputs.markers.push_back(marker);
        }
        trajectoryLinePub.publish(outputs);
    }

    void publishTrajectoryCleanedArea(visualization_msgs::MarkerArray& input)
    {
        visualization_msgs::MarkerArray outputs;

        int id = 0;
        for( visualization_msgs::Marker marker : input.markers)
        {
            marker.id = id++;
            marker.header.seq = this->seq;
            marker.header.stamp = ros::Time::now();
            marker.ns = "cleaned";

            marker.scale.x = 0.3246;
            marker.color.r = 0.0;
            marker.color.g = 0.0;
            marker.color.b = 1.0;
            marker.color.a = 0.1;
            outputs.markers.push_back(marker);
        }
        trajectoryCleanedAreaPub.publish(outputs);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Trajectory_Editor_Node");
    ros::NodeHandle nh;

    CTrajectoryEditorNode teNode(&nh);
    
    teNode.loop();

    return 0;
}
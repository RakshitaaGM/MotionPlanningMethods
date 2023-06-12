#include<iostream>
#include<ros/ros.h>
#include<std_msgs/Header.h>
#include <geometry_msgs/Twist.h>
#include<nav_msgs/OccupancyGrid.h>
#include<geometry_msgs/PoseWithCovarianceStamped.h>
#include<geometry_msgs/PoseStamped.h>
#include<visualization_msgs/Marker.h>
#include "Astar.h"

void mapCallback(const nav_msgs::OccupancyGrid& msg)
{
    std_msgs::Header header = msg.header;
    nav_msgs::MapMetaData mapInfo = msg.info;
    uint height = mapInfo.height;
    uint width = mapInfo.width;
    std::cout << "height " << height << "width " << width <<std::endl;
    // int grid[height][width];
    std::cout << msg.data.size() << std::endl;
    std::cout << "astar is initialised" << std::endl;
    for(uint i = 0; i < width; i ++)
    {
        std::vector<int> rows;
        for(uint j = 0; j < height; j++)
        {
            int val = static_cast<int>(msg.data[i * height + j]);
            val = val < 0 ? 100 : val;
            rows.emplace_back(val);
            // std::cout <<  msg.data[i * height + j] << std::endl; 
        }
        // astar.m_grid.emplace_back(rows);
    }

    // pathPlanning::Node start, goal;
    // start.x = 2.5;
    // start.y = 2.5;
    // goal.x = 6.0;
    // goal.y = 6.0;
    // astar.start = start;
    // astar.goal = goal;
    // astar.nrows = width;
    // astar.ncols = height;
    // std::cout << astar.m_grid[0][1] << std::endl;
    // astar.setStartNode(start);

}

void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped msg)
{
    std::cout << "initialPose " << msg.pose.pose.position.x << " " << msg.pose.pose.position.y << std::endl;
}

void goalPoseCallback(const geometry_msgs::PoseStamped msg)
{
    std::cout << "goal pose " << msg.pose.position.x << " " << msg.pose.position.y << std::endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "astar");
    ros::NodeHandle n;
    ros::Subscriber mapSub = n.subscribe("map", 1000, mapCallback);
    ros::Publisher visMarker = n.advertise<visualization_msgs::Marker>("visualization_msgs", 1);
    ros::Subscriber initPoseSub = n.subscribe("initialpose", 10, initialPoseCallback);
    ros::Subscriber goalPoseSub = n.subscribe("move_base_simple/goal", 10, goalPoseCallback);
    ros::Rate r(1);
    uint32_t shape = visualization_msgs::Marker::CUBE;
    pathPlanning::Astar astar;
    astar.findPath();
    // while(ros::ok())
    // {
    //     visualization_msgs::Marker marker;
    //     marker.header.frame_id = "map";
    //     marker.header.stamp = ros::Time::now();
    //     marker.ns = "initial_pose";
    //     marker.id = 0;
    //     marker.type = shape;
    //     marker.action = visualization_msgs::Marker::ADD;
    //     marker.pose.position.x = 2.5;
    //     marker.pose.position.y = 2.5;
    //     marker.pose.position.z = 0.0;
    //     marker.pose.orientation.x = 0.0;
    //     marker.pose.orientation.y = 0.0;
    //     marker.pose.orientation.z = 0.0;
    //     marker.pose.orientation.w = 1.0;
    //     marker.scale.x = 1.0;
    //     marker.scale.y = 1.0;
    //     marker.scale.z = 1.0;
    //     marker.color.a = 1.0; // Don't forget to set the alpha!
    //     marker.color.r = 0.0f;
    //     marker.color.g = 1.0f;
    //     marker.color.b = 0.0f;
    //     marker.lifetime = ros::Duration();
    //     visMarker.publish( marker );
    //     r.sleep();

    // }
    

    ros::spin();

    return 0;
}

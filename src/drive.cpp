#include<iostream>
#include<ros/ros.h>
#include<std_msgs/Header.h>
#include <geometry_msgs/Twist.h>
#include<nav_msgs/OccupancyGrid.h>
#include "Astar.h"

void mapCallback(const nav_msgs::OccupancyGrid& msg)
{
    std_msgs::Header header = msg.header;
    nav_msgs::MapMetaData mapInfo = msg.info;
    for(uint x = 0; x < mapInfo.width ; x++)
    {
        for(uint y = 0; y < mapInfo.height; y++)
        {
            std::cout << int(msg.data[x + mapInfo.width * y]);
        }
    }
    

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "astar");
    ros::NodeHandle n;
    ros::Subscriber mapSub = n.subscribe("map", 1000, mapCallback);
    ros::spin();

    return 0;
}
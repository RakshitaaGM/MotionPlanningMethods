#include<iostream>
#include<ros/ros.h>
#include<std_msgs/Header.h>
#include <geometry_msgs/Twist.h>
#include<nav_msgs/OccupancyGrid.h>
#include<nav_msgs/Path.h>
#include<geometry_msgs/PoseWithCovarianceStamped.h>
#include<geometry_msgs/PoseStamped.h>
#include<visualization_msgs/Marker.h>
#include<geometry_msgs/Point.h>
#include "Astar.h"
#include "Astar.cpp"
pathPlanning::Astar astar;
std::vector<std::vector<int>> grid;
bool mapFlag;
bool startFlag;

void mapCallback(const nav_msgs::OccupancyGrid& msg)
{
    std_msgs::Header header = msg.header;
    nav_msgs::MapMetaData mapInfo = msg.info;
    uint height = mapInfo.height;
    uint width = mapInfo.width;
    std::cout << "height " << height << "width " << width <<std::endl;
    std::cout << "astar is initialised" << std::endl;
    astar.ncols = width;
    astar.nrows = height;
    for(uint i = 0; i < width; i ++)
    {
        std::vector<int> rows;
        for(uint j = 0; j < height; j++)
        {
            int val = static_cast<int>(msg.data[i * height + j]);
            val = val < 0 ? 100 : val;
            rows.emplace_back(val);
            // std::cout << val << "\t" ;
            // std::cout <<  msg.data[i * height + j] << std::endl; 
        }
        grid.emplace_back(rows);
        // astar.m_grid.emplace_back(rows);
    }
    pathPlanning::Node start, goal;
    
    astar.m_grid = grid;
    std::cout << "grid " << astar.m_grid.size() << " " << astar.m_grid[0].size() << std::endl;
    start.x = 3;
    start.y = 2;
    goal.x = 10;
    goal.y = 12;

    astar.start = start;
    astar.goal = goal;
    astar.findPath();
    mapFlag = true;
    std::cout << "map flag in map callback " << mapFlag << std::endl;
    return;
    // astar.findPath();

}

void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped msg)
{
    std::cout << "initialPose " << msg.pose.pose.position.x << " " << msg.pose.pose.position.y << std::endl;
    pathPlanning::Node start;
    start.x = msg.pose.pose.position.x;
    start.y = msg.pose.pose.position.y;
    astar.start = start;
}

void goalPoseCallback(const geometry_msgs::PoseStamped msg)
{
    std::cout << "goal pose " << msg.pose.position.x << " " << msg.pose.position.y << std::endl;
    pathPlanning::Node goal;
    goal.x = msg.pose.position.x;
    goal.y = msg.pose.position.y;
    astar.goal = goal;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "astar");
    ros::NodeHandle n;
    ros::Subscriber mapSub = n.subscribe("map", 1000, mapCallback);
    ros::Publisher visMarker = n.advertise<visualization_msgs::Marker>("visualization_msgs", 1);
    ros::Publisher pathPub = n.advertise<nav_msgs::Path>("path", 10);
    ros::Subscriber initPoseSub = n.subscribe("initialpose", 10, initialPoseCallback);
    ros::Subscriber goalPoseSub = n.subscribe("move_base_simple/goal", 10, goalPoseCallback);
    ros::Rate r(30);
    mapFlag = false;
    std::cout << "astar declared " << std::endl;
    visualization_msgs::Marker points;
    points.header.frame_id = "map";
    points.header.stamp = ros::Time::now();
    points.ns = "path";
    points.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = 1.0;
    points.id = 0;
    points.type = visualization_msgs::Marker::LINE_STRIP;
    points.scale.x = 0.1;
    points.scale.y = 0.1;
    points.color.g = 1.0;
    points.color.a = 1.0;
   
    // std::cout << "grid " << astar.m_grid.size() << " " << astar.m_grid[0].size() << std::endl;
    while(ros::ok())
    {
        std::cout << "map flag in while loop" << mapFlag << std::endl;
        if(mapFlag)
        {
            // 
            std::cout << "grid " << astar.m_grid.size() << " " << astar.m_grid[0].size() << std::endl;
            // astar.findPath();
            if(!astar.finalPath.empty())
            {
                
                nav_msgs::Path path;
                path.header.stamp = ros::Time::now();
                path.header.frame_id = "map";
                path.poses.clear();
                for(uint i = 0; i < astar.finalPath.size(); i++)
                {
                    geometry_msgs::PoseStamped pose_stamped;
                    geometry_msgs::Point p;
                    pose_stamped.header.stamp = ros::Time::now();
                    pose_stamped.header.frame_id = "map";
                    pose_stamped.pose.position.x = astar.finalPath[i].first;
                    pose_stamped.pose.position.y = astar.finalPath[i].second;
                    pose_stamped.pose.position.z = 0;
                    p.x = astar.finalPath[i].first;
                    p.y = astar.finalPath[i].second;
                    p.z = 0;
                    path.poses.push_back(pose_stamped);
                    points.points.push_back(p);
                }
                pathPub.publish(path);
                visMarker.publish(points);
            }
            
            std::cout << "<------------Not empty-------> " << std::endl;
        }
        // mapFlag = true;
        r.sleep();
        ros::spinOnce();
    }
 
//   float f = 0.0;
//    while (ros::ok())
//    {
  
//        visualization_msgs::Marker points, line_strip, line_list;
//        points.header.frame_id = line_strip.header.frame_id = line_list.header.frame_id = "map";
//       points.header.stamp = line_strip.header.stamp = line_list.header.stamp = ros::Time::now();
//        points.ns = line_strip.ns = line_list.ns = "points_and_lines";
//       points.action = line_strip.action = line_list.action = visualization_msgs::Marker::ADD;
//        points.pose.orientation.w = line_strip.pose.orientation.w = line_list.pose.orientation.w = 1.0;
   
   
   
//       points.id = 0;
//        line_strip.id = 1;
//        line_list.id = 2;
   
  
 
//       points.type = visualization_msgs::Marker::POINTS;
//       line_strip.type = visualization_msgs::Marker::LINE_STRIP;
//        line_list.type = visualization_msgs::Marker::LINE_LIST;
  

//        // POINTS markers use x and y scale for width/height respectively
//        points.scale.x = 0.2;
//       points.scale.y = 0.2;
   
//       // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
//        line_strip.scale.x = 0.1;
//       line_list.scale.x = 0.1;

//        // Points are green
//        points.color.g = 1.0f;
//        points.color.a = 1.0;
//     // Line strip is blue
//        line_strip.color.b = 1.0;
//        line_strip.color.a = 1.0;
//     // Line list is red
//      line_list.color.r = 1.0;
//        line_list.color.a = 1.0;
   
   
   
//        // Create the vertices for the points and lines
//        for (uint32_t i = 0; i < 100; ++i)
//        {
//          float y = 5 * sin(f + i / 100.0f * 2 * M_PI);
//          float z = 5 * cos(f + i / 100.0f * 2 * M_PI);
  
//          geometry_msgs::Point p;
//          p.x = (int32_t)i - 50;
//         p.y = y;
//        p.z = z;
  
//         points.points.push_back(p);
//        line_strip.points.push_back(p);
 
//         // The line list needs two points for each line
//         line_list.points.push_back(p);
//       p.z += 1.0;
//         line_list.points.push_back(p);
//      }
 

//      visMarker.publish(points);
//       visMarker.publish(line_strip);
//     visMarker.publish(line_list);
  
//     r.sleep();

//     f += 0.04;
//   }
    

    ros::spin();

    return 0;
}

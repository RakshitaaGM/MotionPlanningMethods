#include<iostream>
#include<ros/ros.h>
#include <geometry_msgs/Twist.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "moveRobot");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    ros::Rate loop_rate(50);

    double linear_speed=0.2;
    double goal_distance=1.0;
    double linear_duration=goal_distance/linear_speed;
    double angular_speed=1.0;
    double goal_angle=0.0;
    double angular_duration=0.0;
    geometry_msgs::Twist twist_msg;
    twist_msg.linear.x=twist_msg.linear.y=twist_msg.linear.z=0;
    twist_msg.angular.x=twist_msg.angular.y=twist_msg.angular.z=0;

    while(ros::ok())
    {
        // twist_msg.linear.x=linear_speed;
        // int ticks=static_cast<int>(linear_duration*rate);
        // for(int j=0;j<ticks;j++)
        // {
        //     cmd_vel_pub.publish(twist_msg);
        //     loop_rate.sleep();
        // }

        // twist_msg.linear.x=0;
        // cmd_vel_pub.publish(twist_msg);
        // ros::Duration(1).sleep();

        // twist_msg.angular.z=angular_speed;
        // ticks=static_cast<int>(goal_angle*rate);
        // for(int j=0;j<ticks;j++)
        // {
        //     cmd_vel_pub.publish(twist_msg);
        //     loop_rate.sleep();
        // }

        // twist_msg.angular.z=0;
        // cmd_vel_pub.publish(twist_msg);
        // ros::Duration(1).sleep();
        geometry_msgs::Twist msg;
        msg.linear.x = 0.2;
        pub.publish(msg);
        loop_rate.sleep();

    }

    // ttwist_msg.linear.x=0;
    // twist_msg.angular.z=0;
    // cmd_vel_pub.publish(twist_msg);
    // ros::Duration(1).sleep();

    return 0;
}
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

int main(int argc , char **argv)
{
    ros::init(argc,argv,"fake_gps_node");
    ros::NodeHandle nh;

    ros::Publisher fake_vo_pub = nh.advertise<nav_msgs::Odometry>("/fake_gps",1000);

    ros::Rate rate(5);

    double x = 0;

    while (ros::ok())
    {
        nav_msgs::Odometry fake_gps_msg;

        fake_gps_msg.header.stamp = ros::Time::now();
        fake_gps_msg.header.frame_id = "world";
        fake_gps_msg.child_frame_id = "odom";
        fake_gps_msg.pose.pose.position.x = x;
        fake_gps_msg.pose.pose.position.y = x*x;
        fake_gps_msg.pose.pose.position.z = 1;

        fake_vo_pub.publish(fake_gps_msg);

        x += 0.1;

        rate.sleep();
    }

    return 0;
    
}
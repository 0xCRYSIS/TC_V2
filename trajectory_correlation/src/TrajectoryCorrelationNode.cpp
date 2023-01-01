#include <ros/ros.h>
#include <iostream>
#include <nav_msgs/Odometry.h>
#include <mutex>
#include "params.h"

class TrajectoryCorrelation : public ParamServer
{
    public:

        ros::Subscriber vo_sub;
        ros::Subscriber Gps_sub;

        std::vector<nav_msgs::Odometry> VO_BUFFER ;
        std::mutex m;

        TrajectoryCorrelation(ros::NodeHandle *nh) : ParamServer(nh)
        {
            vo_sub = nh->subscribe(EstimatedTrajectoryTopic,1000,&TrajectoryCorrelation::VoCallback,this);
            Gps_sub = nh->subscribe(GroundTruthTopic,1000,&TrajectoryCorrelation::GpsCallback,this);

        }

        void VoCallback(const nav_msgs::Odometry &vo_msg)
        {
            // ROS_INFO("VO_x ==> %d",vo_msg.pose.pose.position.x);
            m.lock();
            VO_BUFFER.push_back(vo_msg);
            m.unlock();

            // ROS_INFO("<from VoCallback> VO_BUFFER SIZE ------> %d",VO_BUFFER.size());

            size_callback(VO_BUFFER);
        }

        void GpsCallback(const nav_msgs::Odometry &Gps_msg)
        {
            // ROS_INFO("GPS_x ==> %d",Gps_msg.pose.pose.position.x);

            // ROS_INFO("<from GpsCallback> VO_BUFFER SIZE ------> %d",VO_BUFFER.size());
            
        }

        void size_callback(const std::vector<nav_msgs::Odometry> msg_buffer)
        {
            // ROS_INFO("VO_BUFFER SIZE ------> %d",VO_BUFFER.size());
            m.lock();
            std::cout << "[from size_callback] VO_BUFFER SIZE ------> " << msg_buffer.size() << std::endl;
            m.unlock();
        }

        void print_params()
        {
            std::cout << "+-------------- parameters --------------+" << std::endl;
            std::cout << "GroundTruthTopic : " << GroundTruthTopic << std::endl;
            std::cout << "EstimatedTrajectoryTopic : " << EstimatedTrajectoryTopic << std::endl;
            std::cout << "Offset : " << Offset << std::endl;
            std::cout << "Scale : " << Scale << std::endl;
            std::cout << "MaxDifference : " << MaxDifference << std::endl;
            std::cout << "Save : " << std::boolalpha << Save << std::endl;
            std::cout << "SaveAssociations : " << std::boolalpha << SaveAssociations << std::endl;
            std::cout << "Plot : " << std::boolalpha << Plot << std::endl;
            std::cout << "Verbose : " << std::boolalpha << Verbose << std::endl; 
        }
};


int main(int argc,char **argv)
{
    ros::init(argc,argv,"TrajectoryCorrelation");
    ros::NodeHandle nh;

    TrajectoryCorrelation TrajCor(&nh);

    TrajCor.print_params();

    ROS_INFO("===== HELLO =====");

    // ros::Rate rate(1);
    // while(ros::ok())
    // {
    //     ROS_INFO("[from main func] VO_BUFFER SIZE ------> %d",TrajCor.VO_BUFFER.size());

    //     // TrajCor.size_callback();

    //     // rate.sleep();
    // }

    // TrajCor.size_callback();
    
    ros::spin();
    return 0;
}
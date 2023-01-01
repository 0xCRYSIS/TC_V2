#pragma once

#ifndef _UTILITY_TRAJECTORY_CORRELATION_H_
#define _UTILITY_TRAJECTORY_CORRELATION_H_

class ParamServer
{
    public:

        std::string GroundTruthTopic;
        std::string EstimatedTrajectoryTopic;
        double Offset;
        double Scale;
        double MaxDifference;
        bool Save;
        bool SaveAssociations;
        bool Plot;
        bool Verbose;
        
        // std::vector<nav_msgs::Odometry> VO_BUFFER , GPS_BUFFER ;

        ParamServer(ros::NodeHandle *nh)
        {
            nh->param<std::string>("/GroundTruthTopic",GroundTruthTopic,"/defauly_gt_topic");
            nh->param<std::string>("/EstimatedTrajectoryTopic",EstimatedTrajectoryTopic,"/default_et_topic");
            nh->param<double>("/Offset",Offset,0.0);
            nh->param<double>("/Scale",Scale,0.0);
            nh->param<double>("/MaxDifference",MaxDifference,0.0);
            nh->param<bool>("/Save",Save,false);
            nh->param<bool>("/SaveAssociations",SaveAssociations,false);
            nh->param<bool>("/Plot",Plot,false);
            nh->param<bool>("/Verbose",Verbose,false);

        }
};

#endif
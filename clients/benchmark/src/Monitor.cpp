/***************************************************************************************************
 * Copyright Five AI Limited 2021
 * All rights reserved

 * This file is released under a Creative Commons Attribution-NonCommercial-ShareAlike 4.0
 * International License. Please refer to the README.md file supplied with this repository.
 ***************************************************************************************************/

#include "benchmark/Util.h"

#include <ros/subscriber.h>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/this_node.h>
#include <ros/param.h>
#include <std_msgs/String.h>

/**
 * ROS node that acts as a barrier waiting for subscribers to complete.
 * Its aggregated to the ROS graph as a required ROS node - see launch.xml.
 * Its purpose is to force killing the ROS graph should the subscribers or
 * publishers refuse to shutdown politely.
*/

struct Params
{
    Params() :
        subCount{-1}
    {}

    int subCount;
};

namespace 
{
    Params params;

    int s_subFinishCount = 0;

    std::mutex s_mutex;
    std::condition_variable s_cv;
    bool s_allSubsFinished = false;
}

void callback(const std_msgs::String& subName)
{
    ROS_INFO_STREAM("Node [" << subName << "] finished");
    
    ++s_subFinishCount;
    if (s_subFinishCount == params.subCount)
    {
        {
            std::lock_guard<std::mutex> lk(s_mutex);
            s_allSubsFinished = true;
        }
        ROS_INFO_STREAM("All " << params.subCount << " subscribers finished");
        s_cv.notify_one();
    }
}

ros::Subscriber makeSubscriber(ros::NodeHandle& nh)
{
    ros::Subscriber sub = nh.subscribe("/monitor_signal", 5, callback);
    return sub;
}

void loadParams(ros::NodeHandle& nh)
{
    params.subCount = getParam<int>(nh, "/sub_count");
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "monitor");
    ros::NodeHandle nh{"~"};

    loadParams(nh);

    ros::Subscriber sub = makeSubscriber(nh);
    
    ros::AsyncSpinner spinner{1}; 
    spinner.start();

    ROS_INFO_STREAM("Waiting for " << params.subCount << " subscribers to finish...");
    {
        std::unique_lock<std::mutex> lk(s_mutex);
        s_cv.wait(lk, []{return s_allSubsFinished;});
    }

    ROS_INFO_STREAM("Stopping the monitor spinner...");
    spinner.stop();

    return 0;
}

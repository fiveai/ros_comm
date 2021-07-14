/***************************************************************************************************
 * Copyright Five AI 2021.
 * All rights reserved.
 ***************************************************************************************************/

#include "msgs/ShmImage.h"
#include "msgs/ShmSharedPtr.h"
#include "util/Types.h"

#include "benchmark/Util.h"
#include "msgs/tzc_Image.h"
#include "tzc/tzc_subscriber.h"

#include <ros/subscriber.h>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/this_node.h>
#include <ros/param.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/String.h>

#include <boost/algorithm/string/replace.hpp>
#include <boost/variant.hpp>

#include <fstream>
#include <thread>

using SharedPtrConstShmImage = fiveai::std_msgs::shm::SharedPtrConstShmImage;
using SharedPtrConstImage = sensor_msgs::ImageConstPtr;
using SizePixels = fiveai::util::SizePixels;

struct Data
{
    Data() :
        latency{0}
    {}

    ros::Duration latency;
};

struct Params
{
    Params() :
        transport{"UNKNOWN"},
        imageCount{-1},
        imageSize{0,0},
        queueSize{-1},
        outputStats{true},
        statsFullFilePath{"UNKNOWN"},
        tcpNoDelay{false},
        pubCount{0},
        subCount{0},
        id{-1},
        extraDelay{0},
        enableSynchStartup{false}
    {}

    std::string transport;
    int         imageCount;
    SizePixels  imageSize; 
    int         queueSize;
    bool        outputStats;
    std::string statsFullFilePath;
    bool        tcpNoDelay;
    int         pubCount;
    int         subCount;
    int         id;
    std::chrono::milliseconds extraDelay;
    bool        enableSynchStartup;
};

namespace 
{
    Params params;

    // each publisher gets its own stats
    std::vector<std::vector<Data>> s_stats;
    int s_receivedImageCount = 0;

    std::mutex s_mutex;
    std::condition_variable s_cv;
    bool s_allImagesReceived = false;
}

template <typename T>
void callback(T img)
{
    const auto& hdr = img->header;

    const auto latency = ros::Time::now() - hdr.stamp;
    // retrieve the id of the publisher who broadcast this image
    const auto pubId = *reinterpret_cast<const int*>(&img->data[0]);
    s_stats.at(pubId).at(hdr.seq).latency = latency;

    ROS_ERROR_STREAM_COND(params.outputStats, "Latency (" << ros::this_node::getName() << "): " 
                            << latency 
                            << " index " << hdr.seq
                            << " data size: " << img->data.size()
                            << " img_width: " << img->width
                            << " img_height: " << img->height
                            );
    
    ROS_ERROR_STREAM_COND(img->data.size() != params.imageSize.getArea(), 
                          "Received image size does not match the expected size");

    ROS_INFO_STREAM_DELAYED_THROTTLE(2, "[" << ros::this_node::getName() << "] " 
                                            << s_receivedImageCount << " images received so far...");
    ++s_receivedImageCount;

    if (s_receivedImageCount == params.pubCount * params.imageCount)
    {
        ROS_INFO_STREAM("[" << ros::this_node::getName() << "] " 
                            << "All " << s_receivedImageCount << " images have been received!");
        {
            std::lock_guard<std::mutex> lk(s_mutex);
            s_allImagesReceived = true;
        }
        s_cv.notify_one();
    }
}

boost::variant<ros::Subscriber, tzc_transport::Subscriber< tzc_transport::sensor_msgs::Image >>
makeSubscriber(ros::NodeHandle& nh)
{
    if (params.transport == "shm")
    {
        auto cb = callback<SharedPtrConstShmImage>;
        return nh.subscribe<SharedPtrConstShmImage>("/image", params.queueSize, cb);
    }
    else if (params.transport == "tcp")
    {
        auto cb = callback<const SharedPtrConstImage&>;
        return nh.subscribe<sensor_msgs::Image>("/image", params.queueSize, cb,
                                                ros::TransportHints{}.tcp()
                                                                     .tcpNoDelay(params.tcpNoDelay));
    }
    else if (params.transport == "udp")
    {
        auto cb = callback<const SharedPtrConstImage&>;
        return nh.subscribe<sensor_msgs::Image>("/image", params.queueSize, cb, ros::TransportHints{}.udp());
    }
    else if (params.transport == "tzc")
    {
        auto cb = callback<const tzc_transport::sensor_msgs::Image::ConstPtr&>;
        using Img = tzc_transport::sensor_msgs::Image;
        tzc_transport::Topic t(nh);
        return t.subscribe<Img>("/image", params.queueSize, cb,
                                ros::TransportHints{}.tcp()
                                                     .tcpNoDelay(params.tcpNoDelay));
    }

    ROS_ERROR_STREAM(ros::this_node::getName() << " failed to create subscriber for " << params.transport << " protocol");
    return {};
}

void dumpStats(const std::string& statsFullFilePath)
{
    for (int i = 0; i < s_stats.size(); ++i)
    {
        auto& subStats = s_stats[i];

        // do not add the publisher id unless multiple publisher are active
        auto fullFilePath = statsFullFilePath;
        if (s_stats.size() > 1)
        {
            boost::algorithm::replace_last(fullFilePath, ".txt", 
                                                         "_pub" + std::to_string(i+1) + ".txt");
        }

        std::ofstream of(fullFilePath);
        for (const auto& st : subStats)
        {
            of << st.latency << std::endl;
        }
    }
}

void loadParams(ros::NodeHandle& nh)
{
    const auto transport = getParam<std::string>(nh, "/transport");
    throwIfTransportNotSupported(transport);

    params.transport = transport;
    params.imageCount = getParam<int>(nh, "/sub/expected_image_count");

    params.queueSize = getParam<int>(nh, "/sub/queue_size");
    params.outputStats = getParam<bool>(nh, "/sub/output_stats");
    params.statsFullFilePath = getParam<std::string>(nh, "stats_full_file_path");

    params.imageSize = SizePixels{static_cast<unsigned int>(getParam<int>(nh, "/image_width_pixels")), 
                                  static_cast<unsigned int>(getParam<int>(nh, "/image_height_pixels"))};

    params.tcpNoDelay = getParam<bool>(nh, "/sub/tcp_no_delay");
    params.pubCount = getParam<int>(nh, "/pub_count");
    params.subCount = getParam<int>(nh, "/sub_count");
    params.id = getParam<int>(nh, "id");
    params.extraDelay = std::chrono::milliseconds{getParam<int>(nh, "/sub/extra_delay_ms")};

    params.enableSynchStartup = getParam<bool>(nh, "/sub/enable_synch_startup");
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "subscriber");
    ros::NodeHandle nh{"~"};

    loadParams(nh);

    // pre-allocate the memory for the s_stats
    s_stats.resize(params.pubCount);
    for (auto& subStats : s_stats)
    {
        subStats.resize(params.imageCount, Data{});
    }

    const auto& myName = ros::this_node::getName();
    if (params.enableSynchStartup)
    {
        if (params.id > 0)
        {
            const std::chrono::seconds delay(params.id *2);
            ROS_INFO_STREAM(myName << " is waiting for extra delay " << delay.count() <<
                            "secs for peer subscriber with id " << params.id -1 << " to come online...");
            std::this_thread::sleep_for(delay);
        }
    }

    if (params.extraDelay > std::chrono::milliseconds::zero())
    {
        ROS_INFO_STREAM(ros::this_node::getName() << " is waiting for extra delay " <<
                        params.extraDelay.count() << " ms...");
        std::this_thread::sleep_for(params.extraDelay);
    }

    auto sub = makeSubscriber(nh);
    ros::Publisher pub = nh.advertise<std_msgs::String>("/monitor_signal", 5);

    ros::AsyncSpinner spinner{1}; 
    spinner.start();

    ROS_INFO_STREAM(myName << " is now ready to receive " << params.pubCount * params.imageCount << " images ...");
    {
        std::unique_lock<std::mutex> lk(s_mutex);
        s_cv.wait(lk, []{return s_allImagesReceived;});
    }

    ROS_INFO_STREAM("Dumping stats to " << params.statsFullFilePath);
    dumpStats(params.statsFullFilePath);
    ROS_INFO_STREAM("Stats successfuly saved.");

    ROS_INFO_STREAM(ros::this_node::getName() << " is notifying the monitor that's finished");
    std_msgs::String tmp;
    tmp.data = myName;
    pub.publish(tmp);

    // wait for the monitor notification to be published
    std::this_thread::sleep_for(std::chrono::seconds(1));

    ROS_INFO_STREAM("Stopping the spinner...");
    spinner.stop();

    return 0;
}

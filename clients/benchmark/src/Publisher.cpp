
#include "benchmark/Util.h"

#include "msgs/ShmImage.h"
#include "msgs/ShmSharedPtr.h"
#include "util/Types.h"
#include "threading/Types.h"
#include "msgs/tzc_Image.h"
#include "tzc/tzc_publisher.h"

#include <ros/publisher.h>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/rate.h>
#include <ros/param.h>
#include <ros/this_node.h>
#include <sensor_msgs/Image.h>

#include <thread>

using SharedPtrConstShmImage = fiveai::std_msgs::shm::SharedPtrConstShmImage;
using SharedPtrShmImage = fiveai::std_msgs::shm::SharedPtrShmImage;
using SharedPtrImage = sensor_msgs::ImagePtr;
using SharedPtrConstImage = sensor_msgs::ImageConstPtr;

using SizePixels = fiveai::platform::util::SizePixels;

struct Params
{
    Params() :
        transport{"UNKNOWN"},
        queueSize{-1},
        imageCount{-1},
        poolSize{-1},
        imageSize{0,0},
        hz{-1},
        subCount{0},
        pubCount{0},
        id{-1},
        extraDelay{0},
        enableSynchStartup{false},
        waitForSubscribers{false},
        shmSizeMegaBytes{0}
    {}

    std::string  transport;
    int          queueSize;
    int          imageCount;
    int          poolSize;
    SizePixels   imageSize;
    double       hz;
    int          subCount;
    int          pubCount;
    int          id;
    std::chrono::milliseconds extraDelay;
    bool         enableSynchStartup;
    bool         waitForSubscribers;
    int          shmSizeMegaBytes;
};

namespace
{
    Params params;
}

inline bool isPoolEnabled()
{
    return params.poolSize > 0;
}

template <typename T> T makeImage();

template <>
SharedPtrShmImage makeImage<SharedPtrShmImage>()
{
    auto& eng = ros::ShmEngine::instance();
    auto img = eng.allocateSharedImage(params.imageSize);
    return img;
}

template <>
SharedPtrImage makeImage<SharedPtrImage>()
{
    SharedPtrImage img = boost::make_shared<sensor_msgs::Image>();

    const auto& imageSize = params.imageSize;

    img->width = imageSize.getWidth();
    img->height = imageSize.getHeight();
    img->step = imageSize.getWidth();
    img->data.resize(imageSize.getArea());

    return img;
}

template <typename T>
std::vector<T> maybeMakePool()
{
    std::vector<T> pool;

    if (isPoolEnabled())
    {
        pool.reserve(params.poolSize);
        ROS_INFO_STREAM("Pool enabled. Preallocating " << params.poolSize << " images");
        for (int i = 0; i < params.poolSize; ++i)
        {
            T img = makeImage<T>();
            pool.push_back(img);
        }
    }
    else
    {
        ROS_INFO_STREAM("Pool disabled. Images will be allocated on the fly");
    }

    return pool;
}

template <typename T, typename U>
void publish(ros::NodeHandle& nh)
{
    if (params.enableSynchStartup)
    {
        if (params.id > 0)
        {
            const std::chrono::seconds delay(params.id *2);
            ROS_INFO_STREAM(ros::this_node::getName() << " is waiting " << delay.count() <<
                            "secs for peer publisher with id " << params.id -1 << " to come online...");
            std::this_thread::sleep_for(delay);
        }
    }

    ros::Publisher pub = nh.advertise<U>("/image", params.queueSize);
    ROS_INFO_STREAM(ros::this_node::getName() << " advertised " << pub.getTopic());

    auto pool = maybeMakePool<T>();

    if (params.extraDelay > std::chrono::milliseconds::zero())
    {
        ROS_INFO_STREAM(ros::this_node::getName() << " is waiting for extra delay " <<
                        params.extraDelay.count() << " ms...");
        std::this_thread::sleep_for(params.extraDelay);
    }

    if (params.waitForSubscribers)
    {
        while (pub.getNumSubscribers() < params.subCount)
        {
            const int pendingSubCount = params.subCount - pub.getNumSubscribers();
            ROS_INFO_STREAM_THROTTLE(2.0, ros::this_node::getName() << " is waiting for " <<
                                          pendingSubCount << " subscribers to come online...");
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    ROS_INFO_STREAM("Start publishing at " << params.hz << "Hz");
    ros::Rate rate{params.hz};
    for (int i = 0; i < params.imageCount; ++i)
    {
        T img;
        if (isPoolEnabled())
        {
            img = pool[i % params.poolSize];
        }
        else
        {
            img = makeImage<T>();
        }

        img->header.seq = i;
        img->header.stamp = ros::Time::now();
        // embed the id of this publisher in the first 4 bytes of the image
        *reinterpret_cast<int*>(&img->data[0]) = params.id;

        pub.publish(img);

        rate.sleep();
    }

    //wait for the messages to be sent to all subscribers before destroying the publisher
    std::this_thread::sleep_for(std::chrono::seconds(5));
}

void publishThroughShm(ros::NodeHandle& nh)
{
    publish<SharedPtrShmImage, SharedPtrConstShmImage>(nh);
}

void publishThroughTcpOrUdp(ros::NodeHandle& nh)
{
    publish<SharedPtrImage, sensor_msgs::Image>(nh);
}

void publishThroughTzc(ros::NodeHandle& nh)
{
    using Img = tzc_transport::sensor_msgs::Image;

    auto megaBytesToBytes = [](std::uint64_t megaBytes){return megaBytes * 1024*1024;};

    if (params.enableSynchStartup)
    {
        if (params.id > 0)
        {
            const std::chrono::seconds delay(params.id *2);
            ROS_INFO_STREAM(ros::this_node::getName() << " is waiting " << delay.count() <<
                            "secs for peer publisher with id " << params.id -1 << " to come online...");
            std::this_thread::sleep_for(delay);
        }
    }

    tzc_transport::Topic t(nh);
    tzc_transport::Publisher<Img> pub = t.advertise<Img>("/image", params.queueSize, megaBytesToBytes(params.shmSizeMegaBytes));
    ROS_INFO_STREAM(ros::this_node::getName() << " advertised " << pub.getTopic());

    if (params.extraDelay > std::chrono::milliseconds::zero())
    {
        ROS_INFO_STREAM(ros::this_node::getName() << " is waiting for extra delay " <<
                        params.extraDelay.count() << " ms...");
        std::this_thread::sleep_for(params.extraDelay);
    }

    if (params.waitForSubscribers)
    {
        while (pub.getNumSubscribers() < params.subCount)
        {
            const int pendingSubCount = params.subCount - pub.getNumSubscribers();
            ROS_INFO_STREAM_THROTTLE(2.0, ros::this_node::getName() << " is waiting for " <<
                                          pendingSubCount << " subscribers to come online...");
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    ROS_INFO_STREAM("Start publishing at " << params.hz << "Hz");
    ros::Rate rate{params.hz};
    const auto& imageSize = params.imageSize;
    for (int i = 0; i < params.imageCount; ++i)
    {
        tzc_transport::sensor_msgs::Image img;

        img.width  = imageSize.getWidth();
        img.height = imageSize.getHeight();
        img.step   = imageSize.getWidth(); //???
        img.data.resize(imageSize.getArea());

        if (pub.allocate(img))
        {
            img.header.seq = i;
            img.header.stamp = ros::Time::now();
            // embed the id of this publisher in the first 4 bytes of the image
            *reinterpret_cast<int*>(&img.data[0]) = params.id;

            pub.publish(img);
        }
        else
        {
            ROS_ERROR_STREAM(ros::this_node::getName() << " failed to allocate image");
        }

        rate.sleep();
    }

    //wait for the messages to be sent to all subscribers before destroying the publisher
    std::this_thread::sleep_for(std::chrono::seconds(5));
}

void loadParams(ros::NodeHandle& nh)
{
    const auto transport = getParam<std::string>(nh, "/transport");
    throwIfTransportNotSupported(transport);

    params.transport = transport;
    params.queueSize = getParam<int>(nh, "/pub/queue_size");
    params.imageCount = getParam<int>(nh, "/pub/image_count");
    params.poolSize = getParam<int>(nh, "/pub/pool_size");
    params.imageSize = {static_cast<unsigned int>(getParam<int>(nh, "/image_width_pixels")),
                        static_cast<unsigned int>(getParam<int>(nh, "/image_height_pixels"))};
    params.hz = getParam<double>(nh, "/pub/hz");
    params.subCount = getParam<int>(nh, "/sub_count");
    params.pubCount = getParam<int>(nh, "/pub_count");
    params.id = getParam<int>(nh, "id");
    params.extraDelay = std::chrono::milliseconds{getParam<int>(nh, "/pub/extra_delay_ms")};

    params.enableSynchStartup = getParam<bool>(nh, "/pub/enable_synch_startup");
    params.waitForSubscribers = getParam<bool>(nh, "/pub/wait_for_subscribers");
    params.shmSizeMegaBytes = getParam<int>(nh, "/shm_size_mega_bytes");
}

void validateParams()
{
    if (params.transport == "tzc")
    {
        if (isPoolEnabled())
        {
            const std::string msg{"TZC does not support memory pools"};
            ROS_ERROR_STREAM(ros::this_node::getName() << msg);
            throw std::logic_error{msg};
        }
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "publisher");
    ros::NodeHandle nh{"~"};

    loadParams(nh);
    validateParams();

    if (params.transport == "shm")
    {
        publishThroughShm(nh);
    }
    else if (params.transport == "tcp" || params.transport == "udp")
    {
        publishThroughTcpOrUdp(nh);
    }
    else if (params.transport == "tzc")
    {
        publishThroughTzc(nh);
    }

    return 0;
}

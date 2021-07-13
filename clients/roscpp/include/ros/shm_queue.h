/***************************************************************************************************
 * Copyright Five AI 2021.
 * All rights reserved.
 ***************************************************************************************************/

#pragma once

#include "threading/ShmQueue.h"
#include "threading/ShmCircularQueue.h"
#include "util/ShmSharedPtr.h"
#include "util/ShmString.h"
#include "msgs/ShmImage.h"

#include <boost/variant/variant.hpp>

namespace ros
{
    using ShmCircularQueue = ros::threading::shm::CircularQueue<ros::shm_msgs::SharedPtrConstShmImage>;

    namespace internal
    {
        template <typename S>
        struct Request
        {
            using Service = S;

            std::size_t         id;
            ros::shm::String sourceNodeName;
            ros::shm::String targetNodeName;
        };

        template <typename S>
        struct Response
        {
            using Service = S;

            std::size_t         requestId;
            ros::shm::String sourceNodeName;
            ros::shm::String targetNodeName;
        };
    }

    struct UnregisterSubscriber
    {
        using Request  = internal::Request<UnregisterSubscriber>;
        using Response = internal::Response<UnregisterSubscriber>;
    };

    struct UnregisterPublisher
    {
        using Request  = internal::Request<UnregisterPublisher>;
        using Response = internal::Response<UnregisterPublisher>;
    };

    struct DetachShm
    {
        using Request  = internal::Request<DetachShm>;
        using Response = internal::Response<DetachShm>;
    };


    using ShmEngineItem  = boost::variant
    <
        DetachShm::Request,
        DetachShm::Response,
        UnregisterSubscriber::Request,
        UnregisterSubscriber::Response,
        UnregisterPublisher::Request,
        UnregisterPublisher::Response
    >;
    using ShmEngineQueue = ros::threading::shm::Queue<ShmEngineItem>;
}

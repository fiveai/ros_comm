/***************************************************************************************************
 * Copyright Five AI 2021.
 * All rights reserved.
 ***************************************************************************************************/


#pragma once

#include "util/ShmUniquePtr.h"
#include "util/ShmSharedPtr.h"
#include "msgs/ShmImage.h"
#include "ros/shm_queue.h"

#include <boost/interprocess/managed_shared_memory.hpp>

#include <map>
#include <string>

namespace ros
{
    /**
     * @class ShmPusher is in charge of creating one @class CircularQueue
     * per connected subscriber in the shared memory and informing the corresponding
     * ShmPuller about its existence via the @class ShmEngine. It also exposes
     * the @fn post method that is used to deliver the image to the interested
     * subscribers.
     */
    class ShmPusher
    {
    public:
        using Image               = fiveai::std_msgs::shm::Image;
        using SharedPtrImage      = fiveai::shm::SharedPtr<Image>;
        using UniquePtrImage      = fiveai::shm::UniquePtr<Image>;
        using SharedPtrConstImage = fiveai::shm::SharedPtr<const Image>;
        using UniquePtrConstImage = fiveai::shm::UniquePtr<const Image>;
        using ShmManager          = boost::interprocess::managed_shared_memory;

        ShmPusher(const std::string& topicName, std::size_t shmCircularQueueCapacity,
                  ShmManager& shmManager);

        ShmPusher(const ShmPusher&) = delete;
        ShmPusher(ShmPusher&&) = delete;
        ShmPusher& operator=(const ShmPusher&) = delete;
        ShmPusher& operator=(ShmPusher&&) = delete;

        const std::string& getName() const
        {
            return m_name;
        }
        const std::string& getTopicName() const
        {
            return m_topicName;
        }

        // returns the name of the queue
        std::string createQueue(const std::string& subscriberName);
        void removeQueue(const std::string& subscriberName);

        void post(SharedPtrConstImage img);
        void post(UniquePtrConstImage img);

    private:
        using ShmCircularQueue = ::ros::ShmCircularQueue;
        using ShmQueueShmUniquePtr = typename boost::interprocess::managed_unique_ptr
                                     <
                                          ShmCircularQueue,
                                          boost::interprocess::managed_shared_memory
                                     >::type;
        using SubscriberQueues = std::map<std::string, ShmQueueShmUniquePtr>;

        std::string makeQueueName(const std::string& subscriberName) const;
        ShmQueueShmUniquePtr makeQueue(const std::string& shmQueueName);

    private:
        const std::string               m_name;
        const std::string               m_topicName;
        ShmManager&                     m_shmManager;
        SubscriberQueues                m_subscriberQueues;
        ShmCircularQueue::ElementCount  m_shmCircularQueueCapacity;

    };

}

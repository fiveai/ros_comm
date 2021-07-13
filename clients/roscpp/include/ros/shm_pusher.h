
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
    class ShmPusher
    {
    public:
        using Image               = fiveai::std_msgs::shm::Image;
        using SharedPtrImage      = fiveai::platform::shm::SharedPtr<Image>;
        using UniquePtrImage      = fiveai::platform::shm::UniquePtr<Image>;
        using SharedPtrConstImage = fiveai::platform::shm::SharedPtr<const Image>;
        using UniquePtrConstImage = fiveai::platform::shm::UniquePtr<const Image>;
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

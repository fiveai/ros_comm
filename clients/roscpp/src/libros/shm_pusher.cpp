/***************************************************************************************************
 * Copyright Five AI 2021.
 * All rights reserved.
 ***************************************************************************************************/

#include "ros/shm_pusher.h"

#include <ros/console.h>
#include <ros/this_node.h>
#include "util/ShmPtr.h"
#include "util/Exception.h"
#include "util/String.h"

#include <stdexcept>
#include <string>

namespace ros
{

ShmPusher::ShmPusher(const std::string& topicName, std::size_t shmCircularQueueCapacity,
                     ShmManager& shmManager) :
    m_name{this_node::getName() + "::ShmPusher::" + topicName},
    m_topicName{topicName},
    m_shmManager{shmManager},
    m_subscriberQueues{},
    m_shmCircularQueueCapacity{shmCircularQueueCapacity}
{
}

std::string ShmPusher::makeQueueName(const std::string& subscriberName) const
{
    auto name = this_node::getName() + "_" + subscriberName + "_" + m_topicName;
    return util::portableShmQueueName(name);
}

std::string ShmPusher::createQueue(const std::string& subscriberName)
{
    const auto shmQueueName = makeQueueName(subscriberName);
    ROS_INFO_STREAM(getName() << " is creating queue " << shmQueueName <<
                    " for subscriber " << subscriberName);
    auto q = makeQueue(shmQueueName);
    m_subscriberQueues.emplace(shmQueueName, boost::move(q));
    return shmQueueName;
}

void ShmPusher::removeQueue(const std::string& subscriberName)
{
    ROS_INFO_STREAM(getName() << " is removing queue belonging to subscriber " << subscriberName);
    auto it = m_subscriberQueues.find(makeQueueName(subscriberName));
    if (it == m_subscriberQueues.cend())
    {
        throw std::logic_error{"Could not find queue corresponding to subscriber " + subscriberName};
    }
    m_subscriberQueues.erase(it);
}

ShmPusher::ShmQueueShmUniquePtr
ShmPusher::makeQueue(const std::string& shmQueueName)
{
    using namespace boost::interprocess;
    using namespace ros;

    ROS_INFO_STREAM(getName() << " is creating shm queue " << shmQueueName <<
                    " with capacity " << m_shmCircularQueueCapacity << " in " <<
                    this_node::getName() << " shared memory");

    ShmCircularQueue* q = nullptr;
    try
    {
        q = m_shmManager.construct<ShmCircularQueue>(shmQueueName.c_str())(m_shmCircularQueueCapacity, m_shmManager);
    }
    catch (const interprocess_exception& ex)
    {
        ROS_ERROR_STREAM("Could not create queue " << shmQueueName << ". Boost exception message follows: "
                         << ex.what() << ", lib error code " << ex.get_error_code()
                         << ", system error code " <<  ex.get_native_error());
        throw;
    }

    return shm::makeManagedUniquePtr(q, m_shmManager);
}

void ShmPusher::post(SharedPtrConstImage img)
{
    for (auto& q : m_subscriberQueues)
    {
        try
        {
            const auto queueSize = q.second->put(img);
            constexpr float percent = 80;
            if (queueSize > q.second->getCapacity() * percent / 100.0)
            {
                ROS_WARN_STREAM("The queue " << getName() << " has reached more than " <<
                                percent << "% of its capacity which is " << q.second->getCapacity());
            }
        }
        catch (...)
        {
            ROS_ERROR_STREAM("Could not put image on queue " << q.first << " due to: "
                             << util::currentExceptionInfo());
        }
    }
}

void ShmPusher::post(UniquePtrConstImage img)
{
    auto sharedImg = shm::convertToShared(boost::move(img), m_shmManager);
    post(sharedImg);
}

}

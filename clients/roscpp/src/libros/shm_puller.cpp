/***************************************************************************************************
 * Copyright Five AI 2021.
 * All rights reserved.
 ***************************************************************************************************/

#include "ros/shm_puller.h"

#include <ros/console.h>
#include <ros/assert.h>
#include <ros/this_node.h>
#include "msgs/ShmSharedPtr.h"
#include "msgs/ShmImage.h"
#include "threading/Utils.h"

#include <boost/make_unique.hpp>

#include <stdexcept>
#include <string>
#include <functional>
#include <future>

namespace ros
{
ShmPuller::ShmPuller(const std::string& topicName, const std::string& publisherName,
                     const WorkerAttributes& workerAttributes,
                     ShmManager& shmManger) :
    m_name{this_node::getName() + "::" + publisherName + "::ShmPuller::" + topicName},
    m_shmName{},
    m_shmQueueName{},
    m_topicName{topicName},
    m_publisherName{publisherName},
    m_shmManager{shmManger},
    m_shmQueue{nullptr},
    m_queue{},
    m_state{State::DETACHED},
    m_spin{true},
    m_forward{true},
    m_worker{workerAttributes},
    m_startBarrier{nullptr},
    m_stopBarrier{nullptr},
    m_onStarted{},
    m_onStopped{},
    m_nodeHandle{boost::make_unique<ros::NodeHandle>()},
    m_publisher{makeRosPublisher()}
{
    resetShmData();

    auto printer = [this](const std::string& action)
    {
        ROS_INFO_STREAM(action << " main loop of thread " << getWorkerAttributes().getName()
                        << " (" << m_worker.getOsId() << ")"
                        << " belonging to " << getName() );
    };

    registerOnStarted(std::bind(printer, "Entering"));
    registerOnStarted([this]{waitAt(*m_startBarrier);});
    registerOnStarted([this]{logState();});

    registerOnStopped(std::bind(printer, "Exiting"));
    registerOnStopped([this]{waitAt(*m_stopBarrier);});
}

ShmPuller::~ShmPuller()
{
}

ShmPuller::ShmQueue*
ShmPuller::retrieveShmQueue(const std::string& queueName) const
{
    ROS_INFO_STREAM(getName() << " is retrieving queue " << queueName <<
                    " located in the shm segment " << getShmName());
    auto& mng = const_cast<ShmManager&>(m_shmManager); // find should have been const?
    const auto ret = mng.find<ShmQueue>(queueName.c_str());

    if (ret.first == nullptr)
    {
        const std::string msg = "Could not retrieve queue " + queueName + " from shm segment " + getShmName();
        ROS_ERROR_STREAM(msg);
        throw std::logic_error{msg};
    }

    return ret.first;
}

const ShmPuller::WorkerAttributes&
ShmPuller::getWorkerAttributes() const
{
    return m_worker.getAttributes();
}

void ShmPuller::join()
{
    return m_worker.join();
}

void ShmPuller::notifyToStart()
{
    ROS_INFO_STREAM(getName() << " has been notified to start");
    doStart(0);
}

void ShmPuller::start()
{
    ROS_INFO_STREAM(getName() << " received start request");
    doStart(1);
    waitAt(*m_startBarrier);
    ROS_INFO_STREAM(getName() << " executed start request");
}

void ShmPuller::doStart(std::size_t offset)
{
    makeStartBarrier(offset);
    m_worker.setMainLoop([this]{mainLoop();});
    m_worker.startMainLoop();
}

void ShmPuller::notifyToStop()
{
    ROS_INFO_STREAM(getName() << " has been notified to stop");
    doStop(0);
}

void ShmPuller::doReleaseRosReferences()
{
    ROS_INFO_STREAM(getName() << " is releasing ROS references");
    m_publisher.reset();
    m_nodeHandle.reset();
    m_forward = false;
}

void ShmPuller::releaseRosReferences()
{
    syncExecute([this]{doReleaseRosReferences();});
}

void ShmPuller::stop()
{
    ROS_INFO_STREAM(getName() << " received stop request");

    doStop(1);
    waitAt(*m_stopBarrier);

    join();

    ROS_INFO_STREAM(getName() << " executed stop request");
}

void ShmPuller::asyncAttachToShm()
{
    asyncExecute([this]{attach();});
}

void ShmPuller::attachToShm()
{
    syncExecute([this]{attach();});
}

void ShmPuller::asyncDetachFromShm()
{
    asyncExecute([this]{detach();});
}

void ShmPuller::detachFromShm()
{
    syncExecute([this]{detach();});
}

void ShmPuller::asyncLogState()
{
    asyncExecute([this]{logState();});
}

void ShmPuller::attach()
{
    const std::string action = m_state == State::ATTACHED ? "Reattaching" :
                                                            "attaching";
    ROS_INFO_STREAM(getName() << " is " << action << " to shm " <<
                    getShmName() << " and queue " << getShmQueueName());

    m_shmQueue = retrieveShmQueue(getShmQueueName());

    transitionTo(State::ATTACHED);
}

void ShmPuller::detach()
{
    if (m_state == State::ATTACHED)
    {
        ROS_INFO_STREAM(getName() << " has been requested to detach from shm " <<
                        getShmQueueName() << " stored in " << getShmName() << " shm. " <<
                        m_shmQueue->getSize() << " items will be lost.");

        resetShmData();
        transitionTo(State::DETACHED);
    }
    else
    {
        ROS_INFO_STREAM(getName() << " ignored the detaching request as it has"
                                     " not been attached yet or it has been detached already");
    }
}

void ShmPuller::logState()
{
    switch (m_state)
    {
    case State::ATTACHED:
    {
        ROS_INFO_STREAM(getName() << " current state is ATTACHED");
    }
    break;
    case State::DETACHED:
    {
        ROS_INFO_STREAM(getName() << " current state is DETACHED");
    }
    break;

    default:
    {
        ROS_INFO_STREAM(getName() << " current state is UNKNOWN");
    }
    }
}

void ShmPuller::transitionTo(State state)
{
    m_state = state;
    logState();
}

void ShmPuller::doStop(std::size_t offset)
{
    makeStopBarrier(offset);

    asyncDetachFromShm();
    asyncExecute([this]{m_spin = false;});
}

void ShmPuller::mainLoop()
{
    call(m_onStarted);

    while (m_spin)
    {
        if (m_state == State::ATTACHED)
        {
            //Take one item off the queue and figure out what it is, then
            //repeat until we are told to stop.
            if (auto item = m_shmQueue->waitForAndPop(boost::posix_time::milliseconds(100)))
            {
                auto img = item.value();
                onImage(std::move(img));
            }

            if (! m_queue.isEmpty())
            {
                auto cmd = m_queue.waitAndPop();
                cmd();
            }
        }
        else
        {
            auto cmd = m_queue.waitAndPop();
            cmd();
        }
    }

    call(m_onStopped);
}

void ShmPuller::onImage(SharedPtrConstImage msg)
{
    using namespace boost::interprocess;

    ROS_DEBUG_STREAM("Received " << *msg);

    if (m_forward)
    {
        auto publishablePtr = boost::make_shared<SharedPtrConstImage>(msg);
        m_publisher->publish(publishablePtr);
    }
}

void ShmPuller::resetShmData()
{
    // NB: do not reset m_shmManager here as there might still be images on flight
    //     and a reset would unmap the shared memory from the address space of this
    //     process leading to nasty crashes.

    m_shmName = "UNKNOWN";
    m_shmQueueName = "UNKNOWN";
    m_shmQueue = nullptr;
}

void ShmPuller::registerOnStarted(OnStartedMainLoopCallback callback)
{
    m_onStarted.push_back(callback);
}

void ShmPuller::registerOnStopped(OnStoppedMainLoopCallback callback)
{
    m_onStopped.push_back(callback);
}

template <typename CallbackSet>
void ShmPuller::call(CallbackSet& callbackSet)
{
    for (auto& callback : callbackSet)
    {
        ROS_ASSERT_MSG(!!callback, "Attempt to call empty lambda");
        callback();
    }
}

void ShmPuller::makeBarrier(std::unique_ptr<boost::barrier>& barrier, std::size_t offset) const
{
    const auto barrierCount = offset + 1;

    // NB. due to a bug in boost::barrier contructor it's not possible
    // to pass a more complex lambda - Boost 1.58
    auto onCompletionStart = []()
    {
        ROS_INFO_STREAM("All threads have reached start/stop barrier");
    };

    barrier = boost::make_unique<boost::barrier>(barrierCount, onCompletionStart);
}

void ShmPuller::waitAt(boost::barrier& barrier)
{
    barrier.count_down_and_wait();
}

void ShmPuller::makeStartBarrier(std::size_t offset)
{
    makeBarrier(m_startBarrier, offset);
}

void ShmPuller::makeStopBarrier(std::size_t offset)
{
   makeBarrier(m_stopBarrier, offset);
}

std::unique_ptr<ros::Publisher> ShmPuller::makeRosPublisher()
{
    ROS_INFO_STREAM(getName() << " is creating local private publisher on topic " << m_topicName);
    auto pub = m_nodeHandle->advertise<SharedPtrConstImage>(m_topicName, 5, false, false, true);
    return boost::make_unique<ros::Publisher>(std::move(pub));
}

void ShmPuller::asyncExecute(Command cmd)
{
    m_queue.put(cmd);
}

template <typename Ret, typename Cmd>
Ret ShmPuller::syncExecute(Cmd cmd)
{
    std::promise<Ret> barrier;
    std::future<Ret> future = barrier.get_future();
    asyncExecute([&barrier, cmd]
    {
        using fiveai::platform::threading::executeAndSetBarrier;
        executeAndSetBarrier<Ret>(barrier, cmd);
    });
    return future.get();
}

}

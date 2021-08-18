/***************************************************************************************************
 * Copyright Five AI Limited 2021
 * All rights reserved

 * This file is released under a Creative Commons Attribution-NonCommercial-ShareAlike 4.0
 * International License. Please refer to the README.md file supplied with this repository.
 ***************************************************************************************************/

#pragma once

#include "threading/Thread.h"
#include "threading/Attributes.h"
#include "threading/Queue.h"
#include "msgs/ShmImage.h"
#include "ros/shm_queue.h"
#include <ros/node_handle.h>

#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/thread/barrier.hpp>

#include <functional>
#include <vector>
#include <future>
#include <condition_variable>

namespace ros
{
    /**
     * @class ShmPuller is in charge of retrieving a pointer to the @class CircularQueue
     * (assigned by the corresponding publisher) from the shared memory and spawing an OS
     * thread whose main responsibility is to wait and pop elements from the queue when
     * they become avalable. In addition to the CircularQueue, there is also an internal
     * thread-safe queue that is used by the ShmPuller to attend internal requests, e.g.
     * attach to/detach from the CircularQueue.
     */
    class ShmPuller
    {
    public:
        using OnStartedMainLoopCallback = std::function<void()>;
        using OnStoppedMainLoopCallback = std::function<void()>;

        using Image               = ros::lot_msgs::Image;
        using SharedPtrImage      = ros::shm::SharedPtr<Image>;
        using SharedPtrConstImage = ros::shm::SharedPtr<const Image>;
        using WorkerAttributes    = ros::threading::StandardAttributes;
        using Worker              = ros::threading::StandardThread;
        using ShmManager          = boost::interprocess::managed_shared_memory;

        ShmPuller(const std::string& topicName, const std::string& publisherName,
                  const WorkerAttributes& workerAttributes,
                  ShmManager& shmManger);

        ShmPuller(const ShmPuller&) = delete;
        ShmPuller(ShmPuller&&) = delete;
        ShmPuller& operator=(const ShmPuller&) = delete;
        ShmPuller& operator=(ShmPuller&&) = delete;

        ~ShmPuller();

        const std::string& getName() const
        {
            return m_name;
        }
        const std::string& getShmName() const
        {
            return m_shmName;
        }
        const std::string& getShmQueueName() const
        {
            return m_shmQueueName;
        }
        const std::string& getTopicName() const
        {
            return m_topicName;
        }
        const std::string& getPublisherName() const
        {
            return m_publisherName;
        }

        const WorkerAttributes& getWorkerAttributes() const;

        void setShmName(const std::string& name)
        {
            m_shmName = name;
        }
        void setShmQueueName(const std::string& name)
        {
            m_shmQueueName = name;
        }

        void attachToShm();
        void asyncAttachToShm();
        void detachFromShm();
        void asyncDetachFromShm();
        void asyncLogState();

        void start();
        void notifyToStart();
        void stop();
        void notifyToStop();
        void join();

        void releaseRosReferences();

        void registerOnStarted(OnStartedMainLoopCallback callback);
        void registerOnStopped(OnStoppedMainLoopCallback callback);

    private:
        using ShmQueue = ::ros::ShmCircularQueue;
        using Command = std::function<void()>;
        using Queue = ros::threading::Queue<Command>;

        void mainLoop();
        void attach();
        void detach();
        ShmQueue* retrieveShmQueue(const std::string& queueName) const;

        template <typename CallbackSet>
        void call(CallbackSet& callbackSet);

        void makeBarrier(std::unique_ptr<boost::barrier>& barrier, std::size_t barrierCount) const;
        void makeStartBarrier(std::size_t offset);
        void makeStopBarrier(std::size_t offset);
        void waitAt(boost::barrier& barrier);

        void doStart(std::size_t offset);
        void doStop(std::size_t offset);

        std::unique_ptr<ros::Publisher> makeRosPublisher();

        enum class State
        {
            DETACHED,        // no shm info available
            ATTACHED,        // both shm info and pointer to shm data structures have been retrieved
        };
        void transitionTo(State state);
        void logState();

        void resetShmData();

        void onImage(SharedPtrConstImage msg);
        void doReleaseRosReferences();

        void asyncExecute(Command cmd);
        template <typename Ret = void, typename Cmd = std::function<Ret()>>
        Ret syncExecute(Cmd cmd);

    private:
        const std::string m_name;
        std::string       m_shmName;
        std::string       m_shmQueueName;
        const std::string m_topicName;
        const std::string m_publisherName;

        ShmManager&       m_shmManager;
        ShmQueue*         m_shmQueue;
        Queue             m_queue;
        State             m_state;
        bool              m_spin;
        bool              m_forward;

        Worker            m_worker;
        std::unique_ptr<boost::barrier> m_startBarrier;
        std::unique_ptr<boost::barrier> m_stopBarrier;

        std::vector<OnStartedMainLoopCallback> m_onStarted;
        std::vector<OnStoppedMainLoopCallback> m_onStopped;

        std::unique_ptr<ros::NodeHandle> m_nodeHandle;
        std::unique_ptr<ros::Publisher>  m_publisher;
    };

}

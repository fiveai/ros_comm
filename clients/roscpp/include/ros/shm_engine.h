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
#include "util/Types.h"
#include "util/ShmUniquePtr.h"
#include "util/ShmSharedPtr.h"
#include "msgs/ShmImage.h"
#include "ros/common.h"
#include "ros/shm_queue.h"

#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/thread/barrier.hpp>
#include <boost/variant/variant.hpp>

#include <functional>
#include <vector>
#include <mutex>
#include <atomic>

namespace ros
{
    ROSCPP_DECL void start();
}

namespace ros
{
    class ShmPusher;
    class ShmPuller;

    struct PullerShmInfo
    {
        std::string shmName;
        std::string shmQueueName;
    };

    /**
     * The @class ShmEngine represents the workhorse of the shared memory mechanism.
     * Every ROS node wishing to take advantage of said mechanism incorporates one
     * instance of this class.
     *
     * The @class ShmEngine is in charge of allocating the shared memory segment in OS,
     * and it does so by means of boost::interprocess::managed_shared_memory. All ROS
     * nodes will attempt to either create or open the memory segment identified in
     * the OS by a unique name, e.g. LOTROS. Whilst all ROS nodes are allowed to map
     * the memory segment into their address space, Boost.Interprocess guarantees that
     * the creation of the memory segment is carried out by one single ROS node. It's
     * unspecfied which of the ROS graph nodes creates the segment.
     *
     * In addition to the above, we have extended ROS’s transport protocols by adding
     * LOTROS, under which ROS nodes rely on the XMLRPC protocol and OS sockets to
     * establish connections, and ShmEngine to transmit data. Each ROS node incorporates
     * one ShmEngine. The LOT protocol settings, e.g. the names of ShmQueues, are exchanged
     * via XMLRPC during the connection setup. As mentioned above, all ROS nodes using LOTROS
     * share a common memory segment with configurable size and identified by a unique, well
     * known string in the OS file system. When topics are advertised by the publisher, the
     * @class ShmEngine assigns one @class ShmPusher per topic. When a node subscribes to a topic,
     * a @class ShmQueue is created by the corresponding topic's @class ShmPusher and its name
     * is returned to the subscriber. The subscriber, in turn, creates a @class ShmPuller per
     * publisher and topic the main job of which is to spawn a thread, attach itself to the
     * @class ShmQueue indicated by the publisher and start waiting for incoming items on
     * the queue. Items may be of any shared memory compliant C++ type. These include the @class ShmSharedPtr
     * and @class ShmUniquePtr, two of the sahred memory compliant smart pointers we have defined.
     *
     * The ShmPuller also creates local private topics in order to forward items popped
     * from the ShmQueue to the ROS subscriber using its standard interface. Unlike regular
     * topics, the scope of private topics is limited to the node that created them, i.e.
     * intra-process, and they are not publicly advertised. ShmPuller combines private topics
     * and ROS intra-process message passing by wrapping incoming items in shared_ptrs and
     * publishing them as though they originated from another ROS node.
     *
     */
    class ShmEngine
    {
    public:
        friend void ros::start();

        using OnStartedMainLoopCallback = std::function<void()>;
        using OnStoppedMainLoopCallback = std::function<void()>;

        using Image               = ros::lot_msgs::Image;
        using SharedPtrImage      = ros::shm::SharedPtr<Image>;
        using UniquePtrImage      = ros::shm::UniquePtr<Image>;
        using SharedPtrConstImage = ros::shm::SharedPtr<const Image>;
        using UniquePtrConstImage = ros::shm::UniquePtr<const Image>;
        using ShmManager          = boost::interprocess::managed_shared_memory;
        using ByteCount           = ros::util::ByteCount;
        using SizePixels          = ros::util::SizePixels;
        using WorkerAttributes    = ros::threading::StandardAttributes;
        using Worker              = ros::threading::StandardThread;
        using ShmQueue             = ::ros::ShmEngineQueue;
        using ShmQueueShmUniquePtr = typename boost::interprocess::managed_unique_ptr
                                     <
                                          ShmQueue,
                                          boost::interprocess::managed_shared_memory
                                     >::type;
        using MessageId = std::size_t;
        template <typename T> using ResponseHandler = std::function<void(const T&)>;
        using ResponseType = boost::variant
        <
            ResponseHandler<UnregisterSubscriber::Response>,
            ResponseHandler<UnregisterPublisher::Response>,
            ResponseHandler<DetachShm::Response>
        >;

        ~ShmEngine();

        static ShmEngine& instance();

        const std::string& getName() const
        {
            return m_name;
        }
        const std::string& getShmName() const
        {
            return m_shmName;
        }
        ByteCount getShmSizeBytes() const
        {
            return m_shmSizeBytes;
        }

        UniquePtrImage allocateUniqueImage(SizePixels sz);
        SharedPtrImage allocateSharedImage(SizePixels sz);

        void post(SharedPtrConstImage img, const std::string& topicName);
        void post(UniquePtrConstImage img, const std::string& topicName);

        void maybeCreatePusher(const std::string& topicName);
        void asyncMaybeCreatePusher(const std::string& topicName);
        PullerShmInfo registerSubscriber(const std::string& topicName,
                                         const std::string& subscriberName);
        void asyncRegisterSubscriber(const std::string& topicName,
                                     const std::string& subscriberName);
        void unregisterSubscriber(const std::string& topicName,
                                  const std::string& subscriberName);
        void aynchUnregisterSubscriber(const std::string& topicName,
                                       const std::string& subscriberName);

        ShmPuller& makePuller(const std::string& topicName,
                              const std::string& publisherName);
        void attachPullerToShm(const std::string& topicName,
                               const std::string& publisherName,
                               const PullerShmInfo& shmInfo);
        void unregisterPublisher(const std::string& topicName,
                                 const std::string& publisherName);
        void asyncAttachPullerToShm(const std::string& topicName,
                                    const std::string& publisherName,
                                    const PullerShmInfo& shmInfo);

        void start();
        void notifyToStart();
        void stop();
        void notifyToStop();
        void join();
        void releaseRosReferences();

        const WorkerAttributes& getWorkerAttributes() const
        {
            return m_worker.getAttributes();
        }

    private:
        struct Entry;
        using Command = std::function<void()>;
        using Queue = ros::threading::Queue<Command>;

        ShmEngine(const std::string& name,
                  const std::string& shmName, ByteCount shmSizeBytes);

        ShmManager makeShmManager();

        template <typename C, typename P>
        static typename C::value_type::element_type&
        get(C& container, P predicate);

        template <typename C, typename P>
        static typename C::value_type::element_type*
        find(C& container, P predicate);

        const ShmPusher& getPusher(const std::string& topicName) const
        {
            return const_cast<ShmEngine*>(this)->getPusher(topicName);
        }
        ShmPusher& getPusher(const std::string& topicName);
        const ShmPusher* findPusher(const std::string& topicName) const
        {
            return const_cast<ShmEngine*>(this)->findPusher(topicName);
        }
        ShmPusher* findPusher(const std::string& topicName);

        const ShmPuller& getPuller(const std::string& topicName, const std::string& publisherName) const
        {
            return const_cast<ShmEngine*>(this)->getPuller(topicName, publisherName);
        }
        ShmPuller& getPuller(const std::string& topicName, const std::string& publisherName);
        const ShmPuller* findPuller(const std::string& topicName, const std::string& publisherName) const
        {
            return const_cast<ShmEngine*>(this)->findPuller(topicName, publisherName);
        }
        ShmPuller* findPuller(const std::string& topicName, const std::string& publisherName);

        bool pullerExists(const std::string& topicName,
                          const std::string& publisherName) const
        {
            return findPuller(topicName, publisherName) != nullptr;
        }

        bool pusherExists(const std::string& topicName) const
        {
            return findPusher(topicName) != nullptr;
        }

        void removeShmFromSystem();

        std::string makePortableShmName(const std::string& shmName) const;

        void mainLoop();

        void makeBarrier(std::unique_ptr<boost::barrier>& barrier, std::size_t barrierCount) const;
        void makeStartBarrier(std::size_t offset);
        void makeStopBarrier(std::size_t offset);
        void waitAt(boost::barrier& barrier);

        void doStart(std::size_t offset);
        void doStop(std::size_t offset);

        void registerOnStarted(OnStartedMainLoopCallback callback);
        void registerOnStopped(OnStoppedMainLoopCallback callback);

        template <typename CallbackSet>
        void call(CallbackSet& callbackSet);

        ShmQueueShmUniquePtr makeShmQueue(const std::string& shmQueueName);
        std::string makeShmServiceQueueName(const std::string& nodeName) const;
        ShmQueue* retrieveShmQueue(const std::string& queueName, ShmManager& shmManager) const;

        template <typename Service>
        void sendRequest(const typename Service::Request&,
                         ResponseHandler<typename Service::Response>);
        template <typename M>
        void postToPeer(const M& message);
        //
        template <typename Request>
        void handleRequest (const Request& request);
        template <typename Response>
        void handleResponse(const Response& response);

        void doHandleRequest(const UnregisterSubscriber::Request& request);
        void doHandleRequest(const UnregisterPublisher::Request& request);
        void doHandleRequest(const DetachShm::Request& request);

        template <typename Service>
        void sendRequestToAll(const std::vector<Entry>& entries);

        static std::vector<ShmEngine::Entry>
        getBlackList(const std::vector<Entry>& entries, const std::string& nodeName);

        MessageId nextMessageId();

        ShmQueue* getPeerServiceQueue(const std::string& peerNodeName);

        PullerShmInfo doRegisterSubscriber(const std::string& topicName,
                                           const std::string& subscriberName);
        void doReleaseRosReferences();
        void asyncExecute(Command cmd);
        template <typename Ret = void, typename Cmd = std::function<Ret()>>
        Ret syncExecute(Cmd cmd);

        void onAllResponsesReceived(std::function<void()> callback);
        void asyncOnAllResponsesReceived(std::function<void()> callback);
        void asyncOnDelayElapsed(std::chrono::nanoseconds dur, std::function<void()> callback);

        void stopAllPullers();
        void detachAllPullers();

    private:
        const std::string           m_name;
        const std::string           m_shmName;
        const ByteCount             m_shmSizeBytes;
        ShmManager                  m_shmManager;

        std::vector<std::unique_ptr<ShmPusher>>  m_pushers;
        std::vector<std::unique_ptr<ShmPuller>>  m_pullers;

        bool                                   m_spin;
        Worker                                 m_worker;
        ShmQueueShmUniquePtr                   m_shmServiceQueue;
        Queue                                  m_serviceQueue;
        std::unique_ptr<boost::barrier>        m_startBarrier;
        std::unique_ptr<boost::barrier>        m_stopBarrier;
        std::vector<OnStartedMainLoopCallback> m_onStarted;
        std::vector<OnStoppedMainLoopCallback> m_onStopped;

        std::atomic<MessageId>              m_messageId;
        std::map<MessageId, ResponseType>   m_pendingResponses;

        struct Entry
        {
            std::string topicName;
            std::string nodeName;

            bool operator==(const Entry& rhs) const
            {
                return topicName == rhs.topicName &&
                       nodeName == rhs.nodeName;
            }
            bool operator!=(const Entry& rhs) const
            {
                return !(*this == rhs);
            }

        };

        std::vector<Entry> m_subscribers;
        std::vector<Entry> m_publishers;

        std::chrono::steady_clock::time_point m_referenceTimePoint;
    };

}

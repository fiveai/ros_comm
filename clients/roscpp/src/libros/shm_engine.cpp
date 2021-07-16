/***************************************************************************************************
 * Copyright Five AI 2021.
 * All rights reserved.
 ***************************************************************************************************/

#include "ros/shm_engine.h"

#include <ros/console.h>
#include <ros/assert.h>
#include <ros/this_node.h>
#include "ros/shm_pusher.h"
#include "ros/shm_puller.h"
#include "msgs/ShmSharedPtr.h"
#include "msgs/ShmImage.h"
#include "util/ShmPtr.h"
#include "util/String.h"
#include "threading/Utils.h"

#include <boost/make_unique.hpp>
#include <boost/variant/get.hpp>

#include <stdexcept>
#include <string>
#include <functional>
#include <algorithm>
#include <thread>
#include <chrono>
#include <future>

namespace ros
{

namespace
{
    struct SelectByTopicAndPublisher
    {
        template <typename T>
        bool operator()(const T& item) const
        {
            return item->getTopicName()     == topicName     &&
                   item->getPublisherName() == publisherName;
        }

        const std::string topicName;
        const std::string publisherName;
    };

    struct SelectByTopic
    {
        template <typename T>
        bool operator()(const T& item) const
        {
            return item->getTopicName() == topicName;
        }

        const std::string topicName;
    };
}

ShmEngine::ShmEngine(const std::string& name,
                     const std::string& shmName, util::ByteCount shmSizeBytes) :
    m_name{name},
    m_shmName{makePortableShmName(shmName)},
    m_shmSizeBytes{shmSizeBytes},
    m_shmManager{makeShmManager()},
    m_pushers{},
    m_pullers{},
    m_spin{true},
    m_worker{WorkerAttributes{"ros-shm-eng"}},
    m_shmServiceQueue{makeShmQueue(makeShmServiceQueueName(this_node::getName()) )},
    m_serviceQueue{},
    m_startBarrier{nullptr},
    m_stopBarrier{nullptr},
    m_onStarted{},
    m_onStopped{},
    m_messageId{0},
    m_pendingResponses{},
    m_subscribers{},
    m_publishers{},
    m_referenceTimePoint{std::chrono::steady_clock::duration::zero()}
{
    auto printer = [this](const std::string& action)
    {
        ROS_INFO_STREAM(action << " main loop of thread " << getWorkerAttributes().getName()
                        << " (" << m_worker.getOsId() << ")"
                        << " belonging to " << getName() );
    };

    registerOnStarted(std::bind(printer, "Entering"));
    registerOnStarted([this]{waitAt(*m_startBarrier);});

    registerOnStopped(std::bind(printer, "Exiting"));
    registerOnStopped([this]{waitAt(*m_stopBarrier);});
}

ShmEngine::~ShmEngine()
{
    removeShmFromSystem();
}

ShmEngine::ShmManager
ShmEngine::makeShmManager()
{
    using namespace boost::interprocess;
    const auto shmName = m_shmName.c_str();

    ROS_INFO_STREAM(getName() << " is opening or creating shared memory " <<
                    m_shmName << " of " << m_shmSizeBytes << " bytes");
    ShmManager manager{boost::interprocess::open_or_create, shmName, m_shmSizeBytes};
    return std::move(manager);
}

void ShmEngine::maybeCreatePusher(const std::string& topicName)
{
    // assign one Pusher per topic
    if (! pusherExists(topicName))
    {
        auto pusher = boost::make_unique<ShmPusher>(topicName, 30, m_shmManager);
        m_pushers.push_back(std::move(pusher));
    }
}
void ShmEngine::asyncMaybeCreatePusher(const std::string& topicName)
{
    asyncExecute([this,topicName]{maybeCreatePusher(topicName);});
}

ShmPuller& ShmEngine::makePuller(const std::string& topicName,
                                 const std::string& publisherName)
{
    ROS_INFO_STREAM(getName() << " is creating puller for topic " << topicName
                    << " broadcast by " << publisherName);
    ShmPuller::WorkerAttributes attributes{"rosShmPuller-" + std::to_string(m_pullers.size())};
    auto puller = boost::make_unique<ShmPuller>(topicName, publisherName,
                                                attributes, m_shmManager);
    m_pullers.push_back(std::move(puller));

    return *m_pullers.back();
}

PullerShmInfo ShmEngine::registerSubscriber(const std::string& topicName,
                                            const std::string& subscriberName)
{
    return syncExecute<PullerShmInfo>([this, &topicName, &subscriberName]()
    {
        return doRegisterSubscriber(topicName, subscriberName);
    });
}

void ShmEngine::asyncRegisterSubscriber(const std::string& topicName,
                                        const std::string& subscriberName)
{
    auto cmd = [this, topicName, subscriberName]
    {
       doRegisterSubscriber(topicName, subscriberName);
    };
    asyncExecute(cmd);
}

PullerShmInfo ShmEngine::doRegisterSubscriber(const std::string& topicName,
                                              const std::string& subscriberName)
{
    ROS_INFO_STREAM(getName() << " is registering subscriber " << subscriberName);

    m_subscribers.push_back({topicName, subscriberName});

    ShmPusher& pusher = getPusher(topicName);
    const auto shmQueueName = pusher.createQueue(subscriberName);
    return {m_shmName, shmQueueName};
}

void ShmEngine::unregisterSubscriber(const std::string& topicName,
                                     const std::string& subscriberName)
{
    ROS_INFO_STREAM(getName() << " is unregistering subscriber " << subscriberName);

    m_subscribers.erase(std::remove(m_subscribers.begin(), m_subscribers.end(),
                                    Entry{topicName, subscriberName}) );

    ShmPusher& pusher = getPusher(topicName);
    pusher.removeQueue(subscriberName);
}

void ShmEngine::aynchUnregisterSubscriber(const std::string& topicName,
                                          const std::string& subscriberName)
{
    asyncExecute([this, topicName, subscriberName]
    {
       unregisterSubscriber(topicName, subscriberName);
    });
}

void ShmEngine::attachPullerToShm(const std::string& topicName,
                                  const std::string& publisherName,
                                  const PullerShmInfo& shmInfo)
{
    ROS_INFO_STREAM(getName() << " is attaching puller to shm owned by " << publisherName);

    m_publishers.push_back({topicName, publisherName});

    // Assign one Puller per topic per publisher. Distinct publishers broadcasting
    // on the same topic will have different ShmPullers assigned.
    ShmPuller* puller = nullptr;
    if (! pullerExists(topicName, publisherName))
    {
        puller = &makePuller(topicName, publisherName);
        ROS_INFO_STREAM(getName() << " is starting puller " << puller->getName());
        puller->start();
    }
    else
    {
        puller = &getPuller(topicName, publisherName);
    }

    puller->setShmName(shmInfo.shmName);
    puller->setShmQueueName(shmInfo.shmQueueName);
    puller->attachToShm();
}

void ShmEngine::asyncAttachPullerToShm(const std::string& topicName,
                                       const std::string& publisherName,
                                       const PullerShmInfo& shmInfo)
{
    auto cmd = [this, topicName, publisherName, shmInfo]
    {
        attachPullerToShm(topicName, publisherName, shmInfo);
    };
    asyncExecute(cmd);
}

void ShmEngine::unregisterPublisher(const std::string& topicName,
                                    const std::string& publisherName)
{
    ROS_INFO_STREAM(getName() << " is unregistering publisher " << publisherName);

    m_publishers.erase(std::remove(m_publishers.begin(), m_publishers.end(),
                                   Entry{topicName, publisherName}) );
}

ShmEngine::UniquePtrImage ShmEngine::allocateUniqueImage(SizePixels sz)
{
    using namespace boost::interprocess;

    ROS_DEBUG_STREAM(getName() << " is allocating image ");

    auto segmentManager = m_shmManager.get_segment_manager();
    Image* img = nullptr;
    try
    {
        img = m_shmManager.construct<Image>(anonymous_instance)
                                           (
                                                *segmentManager, 
                                                Image::Header{*segmentManager},
                                                sz.getHeight(), sz.getWidth(),
                                                "rggb", 
                                                false,
                                                sz.getWidth(),
                                                std::vector<uint8_t>{}
                                            );
        img->data.resize(sz.getArea());
    }
    catch (const boost::interprocess::interprocess_exception& ex)
    {
        ROS_ERROR_STREAM("Could not allocate image. Boost exception message follows: "
                         << ex.what() << ", lib error code " << ex.get_error_code()
                         << ", system error code " <<  ex.get_native_error());
        throw;
    }

    return shm::makeManagedUniquePtr(img, m_shmManager);
}

ShmEngine::SharedPtrImage ShmEngine::allocateSharedImage(SizePixels sz)
{
    return shm::convertToShared(allocateUniqueImage(sz), m_shmManager);
}

template <typename C, typename P>
typename C::value_type::element_type*
ShmEngine::find(C& container, P predicate)
{
    auto it = std::find_if(std::begin(container), std::end(container), predicate);
    if (it == std::end(container))
    {
        return nullptr;
    }

    return it->get();
}

template <typename C, typename P>
inline
typename C::value_type::element_type&
ShmEngine::get(C& container, P predicate)
{
    if (auto* p = find(container, predicate))
    {
        return *p;
    }
    throw std::logic_error{std::string("Could not find item in ") + typeid(container).name()};
}

ShmPusher* ShmEngine::findPusher(const std::string& topicName)
{
    return find(m_pushers, SelectByTopic{topicName});
}

ShmPuller* ShmEngine::findPuller(const std::string& topicName, const std::string& publisherName)
{
    return find(m_pullers, SelectByTopicAndPublisher{topicName, publisherName});
}

ShmPuller& ShmEngine::getPuller(const std::string& topicName, const std::string& publisherName)
{
    return get(m_pullers, SelectByTopicAndPublisher{topicName, publisherName});
}

ShmPusher& ShmEngine::getPusher(const std::string& topicName)
{
    return get(m_pushers, SelectByTopic{topicName});
}

void ShmEngine::post(SharedPtrConstImage img, const std::string& topicName)
{
    asyncExecute([this, img, topicName]
    {
        auto& pusher = getPusher(topicName);
        pusher.post(std::move(img));
    });
}

void ShmEngine::post(UniquePtrConstImage /*img*/, const std::string& /*topicName*/)
{
    //asyncExecute([this, img, topicName]
    //{
    //    auto& pusher = getPusher(topicName);
    //    pusher.post(std::move(img));
    //});
}

void ShmEngine::removeShmFromSystem()
{
    ROS_INFO_STREAM(getName() << " is removing " << m_shmName << " from system");
    const bool ok = boost::interprocess::shared_memory_object::remove(m_shmName.c_str());
    const auto status = ok ? "succeeded" :
                             "failed";
    const std::string extraClarification = ok ?
            "" :
            ". This indicates that the shared memory segment is still being used "
            "by other processes or it did not exist at the time of removal. Both "
            "situations do not indicate incorrect operation, do not panic!";
    ROS_INFO_STREAM("Removing " << m_shmName << " from system has " << status << extraClarification);
}

void ShmEngine::releaseRosReferences()
{
    syncExecute([this]{doReleaseRosReferences();});
}

void ShmEngine::doReleaseRosReferences()
{
    ROS_INFO_STREAM(getName() << " is stopping internal ROS references");
    for (auto& puller : m_pullers)
    {
        puller->releaseRosReferences();
    }
}

std::string ShmEngine::makePortableShmName(const std::string& shmName) const
{
    const auto portableShmName = util::portableShmName(shmName);

    if (portableShmName != shmName)
    {
        ROS_INFO_STREAM(getName() << " made shm name " << shmName << " portable " << portableShmName);
    }

    return portableShmName;
}

std::string ShmEngine::makeShmServiceQueueName(const std::string& nodeName) const
{
    return util::portableShmQueueName(nodeName + "-shm-eng");
}

ShmEngine::ShmQueueShmUniquePtr
ShmEngine::makeShmQueue(const std::string& shmQueueName)
{
    using namespace boost::interprocess;

    ROS_INFO_STREAM(getName() << " is creating shm queue " << shmQueueName
                    << " in " << getShmName() << " shared memory");

    ShmQueue* q = nullptr;
    try
    {
        q = m_shmManager.construct<ShmQueue>(shmQueueName.c_str())(m_shmManager);
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

ShmEngine::ShmQueue*
ShmEngine::retrieveShmQueue(const std::string& queueName, ShmManager& shmManager) const
{
    ROS_INFO_STREAM(getName() << " is retrieving peer service queue " << queueName);
    const auto ret = shmManager.find<ShmQueue>(queueName.c_str());

    if (ret.first == nullptr)
    {
        const std::string msg = "Could not retrieve peer service queue " + queueName;
        ROS_ERROR_STREAM(msg);
        throw std::logic_error{msg};
    }

    return ret.first;
}

void ShmEngine::registerOnStarted(OnStartedMainLoopCallback callback)
{
    m_onStarted.push_back(callback);
}

void ShmEngine::registerOnStopped(OnStoppedMainLoopCallback callback)
{
    m_onStopped.push_back(callback);
}

void ShmEngine::join()
{
    return m_worker.join();
}

void ShmEngine::notifyToStart()
{
    ROS_INFO_STREAM(getName() << " has been notified to start");
    doStart(0);
}

void ShmEngine::start()
{
    ROS_INFO_STREAM(getName() << " received start request");
    doStart(1);
    waitAt(*m_startBarrier);
    ROS_INFO_STREAM(getName() << " executed start request");
}

void ShmEngine::notifyToStop()
{
    ROS_INFO_STREAM(getName() << " has been notified to stop");
    doStop(0);
}

void ShmEngine::stop()
{
    ROS_INFO_STREAM(getName() << " received stop request");

    doStop(1);
    waitAt(*m_stopBarrier);

    join();

    ROS_INFO_STREAM(getName() << " executed stop request");
}

void ShmEngine::doStop(std::size_t offset)
{
    makeStopBarrier(offset);

    asyncExecute([this]
    {
        ROS_INFO_STREAM(getName() << " is detaching all pullers");
        detachAllPullers();
    });

    asyncExecute([this]
    {
        ROS_INFO_STREAM(getName() << " is notifying peers to detach from the shm owned by this node");
        sendRequestToAll<DetachShm>(m_subscribers);
    });

    asyncOnAllResponsesReceived([this]
    {
        ROS_INFO_STREAM("All peers have successfuly detached from " << getName() << "'s shm");
        // tell all shm publishers to cease considering this node as one of their shm subscribers
        asyncExecute([this]
        {
            sendRequestToAll<UnregisterSubscriber>(m_publishers);
        });

        // tell all shm subscribers to cease considering this node as one of their shm publisher
        asyncExecute([this]
        {
            sendRequestToAll<UnregisterPublisher>(m_subscribers);
        });

        asyncOnAllResponsesReceived([this]
        {
            // NB: Here we required a delay to give peers of this ROS node a chance
            //     to send requests and get a response back, and thus avoid hanging.
            std::chrono::milliseconds dur(200);
            ROS_INFO_STREAM("Waiting " << dur.count() << "ms to allow the peers of " <<
                            getName() << " to send requests and receive responses");
            asyncOnDelayElapsed(dur, [this]
            {
                ROS_INFO_STREAM("Stopping all pullers");
                stopAllPullers();

                ROS_INFO_STREAM("Existing main loop");
                m_spin = false;
            });
        });
    });
}

void ShmEngine::asyncOnDelayElapsed(std::chrono::nanoseconds delay, std::function<void()> callback)
{
    using namespace std::chrono;
    if (m_referenceTimePoint.time_since_epoch() == steady_clock::duration::zero())
    {
        m_referenceTimePoint = steady_clock::now();
    }

    const auto elapsed = steady_clock::now() - m_referenceTimePoint;
    if (elapsed < delay)
    {
        ROS_INFO_STREAM_THROTTLE(1, "..." << duration_cast<milliseconds>(delay-elapsed).count() << "ms remaining");
        constexpr milliseconds step{20};
        std::this_thread::sleep_for(step);
        asyncExecute([this, delay, callback]{asyncOnDelayElapsed(delay, callback);});
    }
    else
    {
        callback();
    }
}

void ShmEngine::asyncOnAllResponsesReceived(std::function<void()> callback)
{
    asyncExecute([this, callback]{onAllResponsesReceived(callback);});
}

void ShmEngine::onAllResponsesReceived(std::function<void()> callback)
{
    const auto count = m_pendingResponses.size();
    if (count > 0)
    {
        constexpr std::chrono::milliseconds dur{20};
        ROS_INFO_STREAM_DELAYED_THROTTLE(1, getName() << " is waiting for " << count << " responses from its peers");
        std::this_thread::sleep_for(dur);
        asyncOnAllResponsesReceived(callback);
    }
    else
    {
        ROS_INFO_STREAM("All peers of " << getName() << " have responded.");
        callback();
    }
}

void ShmEngine::doStart(std::size_t offset)
{
    makeStartBarrier(offset);
    m_worker.setMainLoop([this]{mainLoop();});
    m_worker.startMainLoop();
}

void ShmEngine::mainLoop()
{
    call(m_onStarted);

    struct Visitor : public boost::static_visitor<void>
    {
        Visitor(ShmEngine& parent) :
            parent{parent}
        {}

        void operator()(DetachShm::Request& request) const
        {
            parent.handleRequest(request);
        }

        void operator()(DetachShm::Response& response) const
        {
            parent.handleResponse(response);
        }

        void operator()(const UnregisterSubscriber::Request& request) const
        {
            parent.handleRequest(request);
        }

        void operator()(const UnregisterSubscriber::Response& response) const
        {
            parent.handleResponse(response);
        }

        void operator()(const UnregisterPublisher::Request& request) const
        {
            parent.handleRequest(request);
        }

        void operator()(const UnregisterPublisher::Response& response) const
        {
            parent.handleResponse(response);
        }

        ShmEngine& parent;
    };

    Visitor visitor{*this};

    while (m_spin)
    {
        //Take one item off the queue and figure out what it is, then
        //repeat until we are told to stop.
        if (auto item = m_serviceQueue.waitForAndPop(std::chrono::milliseconds(10)))
        {
            auto cmd = item.value();
            cmd();
        }

        if (! m_shmServiceQueue->isEmpty())
        {
            auto item = m_shmServiceQueue->waitAndPop();
            boost::apply_visitor(visitor, item);
        }
    }

    call(m_onStopped);
}

template <typename CallbackSet>
void ShmEngine::call(CallbackSet& callbackSet)
{
    for (auto& callback : callbackSet)
    {
        ROS_ASSERT_MSG(!!callback, "Attempt to call empty lambda");
        callback();
    }
}

void ShmEngine::waitAt(boost::barrier& barrier)
{
    barrier.count_down_and_wait();
}

void ShmEngine::makeStartBarrier(std::size_t offset)
{
    makeBarrier(m_startBarrier, offset);
}

void ShmEngine::makeStopBarrier(std::size_t offset)
{
   makeBarrier(m_stopBarrier, offset);
}

void ShmEngine::makeBarrier(std::unique_ptr<boost::barrier>& barrier, std::size_t offset) const
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

template <typename Service>
void ShmEngine::sendRequest(const typename Service::Request& request,
                            ResponseHandler<typename Service::Response> responseHandler)
{
    ROS_INFO_STREAM(getName() << " is sending request " << typeid(request).name() <<
                    " with reqId=" << request.id << " to " << request.targetNodeName);
    m_pendingResponses.emplace(request.id, responseHandler);
    postToPeer(request);
}

template <typename Service>
void ShmEngine::sendRequestToAll(const std::vector<Entry>& entries)
{
    for (const auto& entry : entries)
    {
        typename Service::Request request
        {
            nextMessageId(),
            shm::makeString(this_node::getName(), m_shmManager),
            shm::makeString(entry.nodeName, m_shmManager)
        };
        sendRequest<Service>(request,
                             [this, entry](const typename Service::Response& response)
                             {
                                ROS_INFO_STREAM(response.sourceNodeName << " has responded to "
                                                << getName() << " with regards to topic "
                                                << entry.topicName);
                             });
    }
}

std::vector<ShmEngine::Entry>
ShmEngine::getBlackList(const std::vector<Entry>& entries, const std::string& nodeName)
{
    std::vector<Entry> blackList;
    auto predicate = [&nodeName](const Entry& entry)
    {
       return entry.nodeName == nodeName;
    };

    std::copy_if(entries.cbegin(), entries.cend(),
                 std::back_inserter(blackList),
                 predicate);

    return blackList;
}

template <typename Request>
void ShmEngine::handleRequest(const Request& request)
{
    ROS_INFO_STREAM(getName() << " is handling request " << typeid(Request).name() <<
                    " identified by reqId=" << request.id << " from " << request.sourceNodeName);

    doHandleRequest(request);
    ROS_INFO_STREAM(getName() << " has handled the request identified by reqId=" << request.id);

    using Service = typename Request::Service;
    typename Service::Response response
    {
        request.id,
        request.targetNodeName,
        request.sourceNodeName
    };

    postToPeer(response);
    ROS_INFO_STREAM(getName() << " has responded to the request identified by reqId=" << request.id);
}

template <typename Response>
void ShmEngine::handleResponse(const Response& response)
{
    ROS_INFO_STREAM(getName() << " is handling response " << typeid(Response).name() <<
                    " corresponding to reqId=" << response.requestId << " from " << response.sourceNodeName );

    auto it = m_pendingResponses.find(response.requestId);
    if (it == m_pendingResponses.end())
    {
        ROS_ASSERT_MSG(false, "No response handle could be found");
    }

    auto& callback = boost::get<ResponseHandler<Response>>(it->second);
    callback(response);

    m_pendingResponses.erase(it);
}

void ShmEngine::doHandleRequest(const UnregisterSubscriber::Request& request)
{
    const auto blackList = getBlackList(m_subscribers, request.sourceNodeName.c_str());
    for (const auto& entry : blackList)
    {
        unregisterSubscriber(entry.topicName, entry.nodeName);
    }
}

void ShmEngine::doHandleRequest(const UnregisterPublisher::Request& request)
{
    const auto blackList = getBlackList(m_publishers, request.sourceNodeName.c_str());
    for (const auto& entry : blackList)
    {
        unregisterPublisher(entry.topicName, entry.nodeName);
    }
}

void ShmEngine::doHandleRequest(const DetachShm::Request& request)
{
    // detach only the pullers assigned to request.sourceNodeName
    for (auto& puller : m_pullers)
    {
        if (puller->getPublisherName() == request.sourceNodeName.c_str())
        {
            puller->detachFromShm();
        }
    }
}

template <typename M>
void ShmEngine::postToPeer(const M& message)
{
    auto peerQueue = getPeerServiceQueue(message.targetNodeName.c_str());
    peerQueue->put(message);
}

ShmEngine::ShmQueue* ShmEngine::getPeerServiceQueue(const std::string& peerNodeName)
{
    const auto peerQueueName = makeShmServiceQueueName(peerNodeName);
    ShmQueue* peerServiceQueue = retrieveShmQueue(peerQueueName, m_shmManager);
    return peerServiceQueue;
}

ShmEngine::MessageId ShmEngine::nextMessageId()
{
    ++m_messageId;
    return m_messageId;
}

void ShmEngine::asyncExecute(Command cmd)
{
    m_serviceQueue.put(cmd);
}

template <typename Ret, typename Cmd>
Ret ShmEngine::syncExecute(Cmd cmd)
{
    std::promise<Ret> barrier;
    std::future<Ret> future = barrier.get_future();
    asyncExecute([&barrier, cmd]
    {
        using threading::executeAndSetBarrier;
        executeAndSetBarrier<Ret>(barrier, cmd);
    });
    return future.get();
}

void ShmEngine::stopAllPullers()
{
    for (auto& puller : m_pullers)
    {
        puller->stop();
    }
}

void ShmEngine::detachAllPullers()
{
    for (auto& puller : m_pullers)
    {
        puller->detachFromShm();
    }
}

}

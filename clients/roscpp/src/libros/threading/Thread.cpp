/***************************************************************************************************
 * Copyright Five AI Limited 2021
 * All rights reserved

 * This file is released under a Creative Commons Attribution-NonCommercial-ShareAlike 4.0
 * International License. Please refer to the README.md file supplied with this repository.
 ***************************************************************************************************/


#include "threading/Thread.h"

#include "threading/AttributesIO.h"
#include "threading/Utils.h"
#include "util/Exception.h"

#include <ros/console.h>
#include <ros/assert.h>

#include <pthread.h>
#include <sys/resource.h>

namespace ros { namespace threading
{

template <typename A>
Thread<A>::Thread() noexcept :
    m_attributes{},
    m_worker{},
    m_mainLoop{}
{}

template <typename A>
Thread<A>::Thread(const Attributes& attributes) :
    m_attributes{attributes},
    m_worker{},
    m_mainLoop{}
{
}

template <typename A>
Thread<A>::Thread(Thread&& other) noexcept :
    Thread{}
{
    this->swap(other);
}

template <typename A>
Thread<A>& Thread<A>::operator=(Thread&& other) noexcept
{
    this->swap(other);
    return *this;
}

template <typename A>
Thread<A>::~Thread()
{
    ROS_INFO_STREAM("Thread " << m_attributes.getName() << " (" << getOsId() << ") is being destroyed");
    if (m_worker.joinable())
    {
        const auto msg = "The thread " + m_attributes.getName() +
            " is still running and needs to be joined prior to its destruction";
        ROS_ASSERT_MSG(false, "%s", msg.c_str());
    }
}

template <typename A>
void Thread<A>::startMainLoop()
{
    const auto offlineAttributes = initOfflineAttributes();

    ROS_INFO_STREAM("Spawning thread with attributes: " << m_attributes);
    m_worker = Worker{offlineAttributes, [this]{run();}};
}

template <typename A>
void Thread<A>::run()
{
    initLiveAttributes();

    ROS_INFO_STREAM("Executing main loop of thread " << m_attributes.getName() << " (" << getOsId() << ")");
    ROS_ASSERT_MSG(m_mainLoop != nullptr, "Main loop function not set");
    m_mainLoop();
    ROS_INFO_STREAM("Main loop of thread " << m_attributes.getName() << " (" << getOsId() << ") exited");
}

template <typename A>
boost::thread::attributes Thread<A>::initOfflineAttributes()
{
    using util::tryInvoke;

    boost::thread::attributes boostAttributes;
    // take a convenient alias
    auto nh = boostAttributes.native_handle();

    ROS_INFO_STREAM("Initialising offline attributes for thread " << m_attributes.getName());

    // scheduling policy
    {
        auto msg = [this]
        {
            return "Could not set scheduling policy of thread " +
                m_attributes.getName() + " to " + m_attributes.getPolicyName();
        };
        using scheduling_policy::valueOf;
        ROS_DEBUG_STREAM("Setting scheduling policy to " << m_attributes.getPolicyName() << " for thread " << m_attributes.getName());
        tryInvoke(::pthread_attr_setschedpolicy, msg, nh, valueOf(m_attributes.getPolicy()));
    }

    // static priority
    {
        auto msg = [this] {return "Could not set inherit-scheduler of thread " + m_attributes.getName();};

        // In order for the parameter setting made by pthread_attr_setsched‐
        // param() to have effect when calling pthread_create(3), the caller
        // must use pthread_attr_setinheritsched(3) to set the inherit-scheduler
        // attribute of the attributes object attr to PTHREAD_EXPLICIT_SCHED.
        tryInvoke(::pthread_attr_setinheritsched, msg, nh, PTHREAD_EXPLICIT_SCHED);

        initStaticPriority(nh);
    }

    // unless we have specific requirements leave CPU affinity untouched
    if (! m_attributes.getCpuAffinities().empty())
    {
        auto msg = [this] {return "Could not set affinity of thread " + m_attributes.getName();};
        cpu_set_t cpuSet;
        CPU_ZERO(&cpuSet);

        for (const auto affinity : m_attributes.getCpuAffinities())
        {
            CPU_SET(affinity.getValue(), &cpuSet);
        }
        ROS_DEBUG_STREAM("Setting CPU affinities to " << m_attributes.getCpuAffinities() << " for thread " << m_attributes.getName());
        tryInvoke(::pthread_attr_setaffinity_np, msg, nh, sizeof(cpu_set_t), &cpuSet);
    }

    // stack size
    boostAttributes.set_stack_size(m_attributes.getStackSize().getValue());

    return boostAttributes;
}

template <>
void Thread<StandardAttributes>::initLiveAttributes()
{
   ROS_DEBUG_STREAM("Initialising live attributes for thread " << m_attributes.getName());

   initCommonLiveAttributes();

    auto msg = [this] {return "Could not set nice of thread " + m_attributes.getName();};
    util::tryInvoke(::setpriority, msg, PRIO_PROCESS, 0, m_attributes.getNice().getValue());
}

template <>
void Thread<RealTimeAttributes>::initLiveAttributes()
{
    initCommonLiveAttributes();
}

template <typename A>
void Thread<A>::initCommonLiveAttributes()
{
    auto msg = [this] {return "Could not set name of thread " + m_attributes.getName();};
    util::tryInvoke(baptizeThisThread, msg, m_attributes.getName());
}

template <>
void Thread<StandardAttributes>::initStaticPriority(pthread_attr_t* nh)
{
    auto msg = [this] {return "Could not set static priority of thread " + m_attributes.getName();};
    struct sched_param p;
    p.sched_priority = 0; //for standard threads this *must* be 0
    ROS_DEBUG_STREAM("Setting static priority to " << p.sched_priority << " for standard thread " << m_attributes.getName());
    util::tryInvoke(::pthread_attr_setschedparam, msg, nh, &p);
}

template <>
void Thread<RealTimeAttributes>::initStaticPriority(pthread_attr_t* nh)
{
    auto msg = [this] {return "Could not set static priority of thread " + m_attributes.getName();};
    struct sched_param p;
    p.sched_priority = m_attributes.getStaticPriority().getValue();
    ROS_DEBUG_STREAM("Setting static priority to " << p.sched_priority << " for real-time thread " << m_attributes.getName());
    util::tryInvoke(::pthread_attr_setschedparam, msg, nh, &p);
}

// explicit instantiation
template class Thread<StandardAttributes>;
template class Thread<RealTimeAttributes>;

}}

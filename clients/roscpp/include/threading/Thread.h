/***************************************************************************************************
 * Copyright Five AI 2021.
 * All rights reserved.
 ***************************************************************************************************/
#pragma once

#include "threading/Attributes.h"

#include <boost/thread/thread.hpp>

namespace fiveai { namespace threading
{
    template <typename A>
    class Thread
    {
    public:
        using Attributes = A;
        using Worker = boost::thread;

        Thread() noexcept;

        ~Thread();

        Thread(const Thread&) = delete;
        Thread& operator=(const Thread& other) = delete;

        Thread(Thread&& other) noexcept;
        Thread& operator=(Thread&& other) noexcept;

        explicit Thread(const Attributes& attributes);

        template <typename F, typename... Args>
        explicit Thread(const Attributes& attributes, F&& mainLoop, Args&&... args) :
            m_attributes{attributes},
            m_worker{},
            m_mainLoop{std::bind(std::forward<F>(mainLoop), std::forward<Args>(args)...)}
        {
        }

        template <typename F, typename... Args>
        void setMainLoop(F&& mainLoop, Args&&... args)
        {
            m_mainLoop = std::bind(std::forward<F>(mainLoop), std::forward<Args>(args)...);
        }

        void startMainLoop();

        const Attributes& getAttributes() const
        {
            return m_attributes;
        }

        boost::thread::id getOsId() const noexcept
        {
            return m_worker.get_id();
        }

        bool joinable() const
        {
            return m_worker.joinable();
        }

        void join()
        {
            return m_worker.join();
        }

        template <class Rep, class Period>
        bool tryJoinFor(const boost::chrono::duration<Rep, Period>& dur) const
        {
            return m_worker.try_join_for(dur);
        }

        template <class Clock, class Duration>
        bool tryJoinUntil(const boost::chrono::time_point<Clock, Duration>& tp)
        {
            return m_worker.try_join_until(tp);
        }

        void detach()
        {
            m_worker.detach();
        }

        void interrupt()
        {
            m_worker.interrupt();
        }

        static unsigned hardwareConcurrency() noexcept
        {
            return Worker::hardware_concurrency();
        }

        boost::thread::native_handle_type nativeHandle()
        {
            return m_worker.native_handle();
        }

        void swap(Thread& th) noexcept
        {
            using std::swap;
            swap(m_attributes, th.m_attributes);
            swap(m_worker,     th.m_worker);
            swap(m_mainLoop,   th.m_mainLoop);
        }

    private:
        boost::thread::attributes initOfflineAttributes();
        void initLiveAttributes();
        void initCommonLiveAttributes();
        void initStaticPriority(pthread_attr_t* nh);
        void run();

    private:
        Attributes              m_attributes;
        Worker                  m_worker;
        std::function<void()>   m_mainLoop;
    };

    using StandardThread = Thread<StandardAttributes>;
    using RealTimeThread = Thread<RealTimeAttributes>;
}}

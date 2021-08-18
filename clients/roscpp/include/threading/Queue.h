/***************************************************************************************************
 * Copyright Five AI Limited 2021
 * All rights reserved

 * This file is released under a Creative Commons Attribution-NonCommercial-ShareAlike 4.0
 * International License. Please refer to the README.md file supplied with this repository.
 ***************************************************************************************************/

#pragma once

#include <boost/optional.hpp>

#include <chrono>
#include <functional>
#include <memory>
#include <deque>
#include <mutex>
#include <condition_variable>

namespace ros { namespace threading
{
    /**
     * The @class Queue is a lock based data structure which provides
     * thread-safe access to the data it owns from multiple processes. Specifically,
     * instances of @class Queue can be placed in OS shared memory, and elemets can
     * be put and popped on them concurently from multiple threads spread across
     * various processes.
     *
     * In this context, thread safeness refers to the fact that multiple threads can
     * access the data owned by instances of @class Queue such that the
     * following conditions are met:
     *   - each thread sees a consistent view of the data
     *   - no data is lost or corrupted
     *   - no race conditions arise
     *   - threads can perform the same or distinct operations
     *
     * All the methods exposed by the class are safe-thread and can be called from
     * different processes on the same instance or different instances of @class Queue,
     * unless otherwise specified.
     *
     * The layout of type T @em must be compatible with shared memory, it must not contain
     * raw pointers.
     *
     * For the sake of simplicity, the copy and move semantics have been explicitly disabled,
     * the motivation being that it's quite unusual to copy or move thread-safe queues.
     * However, should these operations be required they can be implemented at a later time.
     *
     * There are two main (thread-safe) operations supported by the class' public interface,
     * *put* and *pop*. The class is implemented in terms of std::dqueue and consequently it
     * exposes a limited functionality of a FIFO like data structure. The 'pop' suite come
     * in two flavours, return-by-value and return-by-non-const-reference. In spite of the
     * fact that the former may incur additional overhead, it should be preffered over the
     * latter overload because it enhances code clarity at the calling site.
     *
     * Warning: the class does not support recurssive calls to its methods from within the
     * same thread and doing so results in undefined behaviour (the underlying mutex which
     * protects the data is @em not recursive).
    */
    template <typename T>
    class Queue
    {
    public:
        using Element = T;
        using Container         = std::deque<Element>;
        using Allocator         = typename Container::allocator_type;
        using Mutex             = std::mutex;
        using ConditionVariable = std::condition_variable;

        Queue() :
            m_allocator{},
            m_data(m_allocator), // use () or else very nasty compile time errors occur
            m_mutex{},
            m_cv{}
        {}

        /*
         * Disable copy and move semantics.
        */
        Queue(const Queue&) = delete;
        Queue(Queue&&) = delete;
        Queue& operator=(const Queue&) = delete;
        Queue& operator=(Queue&&) = delete;

        /*
         * The block of comments applies to all 'put' methods.
         *
         * Once the element has been pushed onto the internal queue, all the threads waiting
         * on this queue are notified. It is unspecified which waiting thread will be
         * awaken up.
        */

        /*
         * Puts an rvalue element onto the queue, i.e. the element gets moved.
        */
        void put(Element&& value)
        {
            auto lk{makeLock()};
            m_data.push_back(std::move(value));
            m_cv.notify_all();
        }

        /*
         * Puts an lvalue element onto the queue, i.e. the element gets copied.
        */
        void put(const Element& value)
        {
            auto lk{makeLock()};
            m_data.push_back(value);
            m_cv.notify_all();
        }
        /*
         * Emplaces a new element onto the queue, i.e. the element gets constructed
         * in-place no copy or move operation are performed.
        */
        template <typename... Args>
        void putEmplace(Args&&... args)
        {
            auto lk{makeLock()};
            m_data.emplace_back(std::forward<Args>(args)...);
            m_cv.notify_all();
        }

        /*
         * If the queue is empty, the calling thread will block until an element
         * is pushed onto the queue. Otherwise, the first element is taken off the
         * queue and std::move-d into the supplied parameter.
         *
         * @param value the next element from the queue
        */
        void waitAndPop(Element& value)
        {
            auto lk{makeLock()};
            m_cv.wait(lk, [this]{return !m_data.empty();});
            value = std::move(m_data.front());
            m_data.pop_front();
        }

        /*
         * If the queue is empty, the calling thread will block until an element
         * is pushed onto the queue. Otherwise, the first element is taken off the
         * queue and std::move-d into a new element constructed on the stack.
         * Eventually the stack element is returned.
         *
         * Note, although very convenient this method may incur substantial overhead
         * for large data types.
         *
         * Note, does not use brace initialization of `value` due to undesired behavior
         *  with std::initializer_list<T>!
         *
         * @return   the first element from the queue
        */
        Element waitAndPop()
        {
            auto lk{makeLock()};
            m_cv.wait(lk, [this]{return !m_data.empty();});
            Element value(std::move(m_data.front()));
            m_data.pop_front();
            return value;
        }

        /*
         * The block of comments applies to both flavours of waitUntilAndPop methods.
         *
         * If the queue is empty, the calling thread will block until an element
         * is pushed onto the queue or the specified time point has been reached.
         * Otherwise, the first element is taken off the queue and std::move-d
         * into a returned element. The way the returned element is delivered back
         * to the caller depends on the flavour of the method being employed, i.e.
         * either by non-const reference or normal return.
         *
         * The boolean flavour is more efficient than its boost::option counterpart,
         * however its usage is ackward. It's recommended that the latter flavour
         * should be used whenever possbile.
         */

        /*
         * Boolean flavour specific comments:
         *
         * @param value  the next element from the queue
         * @pram tp      the time point that needs to reached to unblock the calling thread
         * @return       'true' if the first element has been successfully retrieved from the queue
        */
        bool waitUntilAndPop(Element& value, const std::chrono::steady_clock::time_point tp)
        {
            auto lk{makeLock()};
            const auto isDataAvailable = m_cv.wait_until(lk, tp, [this]{return !m_data.empty();});

            if (! isDataAvailable)
            {
                return false;
            }

            value = std::move(m_data.front());
            m_data.pop_front();

            return true;
        }

        /*
         * boost::optional flavour specific comments:
         *
         * @pram tp  the time point that needs to reached to unblock the calling thread
         * @return   the first Element wrapped in boost::optional which may contain a valid value
         *
         * Note, although very convenient this method may incur substantial overhead
         * for large data types. For small objects the overhead is negligible.
         *
         * Note, does not use brace initialization of `value` due to undesired behavior
         *  with std::initializer_list<T>!
        */
        boost::optional<Element> waitUntilAndPop(const std::chrono::steady_clock::time_point tp)
        {
            auto lk{makeLock()};
            const auto isDataAvailable = m_cv.wait_until(lk, tp, [this]{return !m_data.empty();});
            if (! isDataAvailable)
            {
                return boost::none;
            }

            Element value(std::move(m_data.front()));
            m_data.pop_front();
            return std::move(value);
        }

        /*
         * The same comments as for the waitUntilAndPop methods apply.
         * Convenience methods which take durations rather than time points.
        */
        bool waitForAndPop(Element& value, const std::chrono::milliseconds dur)
        {
            return waitUntilAndPop(value, std::chrono::steady_clock::now() + dur);
        }

        boost::optional<Element> waitForAndPop(const std::chrono::milliseconds dur)
        {
            return waitUntilAndPop(std::chrono::steady_clock::now() + dur);
        }

        /*
         * Checks whether the queue is empty or not.
        */
        bool isEmpty() const
        {
            auto lk{makeLock()};
            return m_data.empty();
        }

        /**
         * @return     The number of elements held by the queue
         */
        typename Container::size_type getSize() const
        {
            auto lk{makeLock()};
            return m_data.size();
        }

    private:
        std::unique_lock<Mutex> makeLock() const
        {
            return std::unique_lock<Mutex>{m_mutex};
        }

    private:
        Allocator                  m_allocator;
        Container                  m_data;
        mutable Mutex              m_mutex;
        mutable ConditionVariable  m_cv;
    };

}}


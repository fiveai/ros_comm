/***************************************************************************************************
 * Copyright Five AI 2021.
 * All rights reserved.
 ***************************************************************************************************/

#pragma once

#include <boost/circular_buffer.hpp>
#include <boost/interprocess/allocators/allocator.hpp>
#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include <boost/interprocess/sync/interprocess_condition.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>
#include <boost/optional.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>

#include <functional>
#include <memory>

namespace ros { namespace threading { namespace shm
{
    /**
     * The @class CircularQueue is a lock based data structure which provides
     * thread-safe access to the data it owns from multiple processes. Specifically,
     * instances of @class CircularQueue can be placed in OS shared memory, and elemets can
     * be put and popped on them concurently from multiple threads spread across
     * various processes.
     *
     * In this context, thread safeness refers to the fact that multiple threads can
     * access the data owned by instances of @class CircularQueue such that the
     * following conditions are met:
     *   - each thread sees a consistent view of the data
     *   - no data is lost or corrupted
     *   - no race conditions arise
     *   - threads can perform the same or distinct operations
     *
     * All the methods exposed by the class are safe-thread and can be called from
     * different processes on the same instance or different instances of @class CircularQueue,
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
    class CircularQueue
    {
    public:
        using Element = T;
        using ShmManager        = boost::interprocess::managed_shared_memory;
        using ShmSegmentManager = ShmManager::segment_manager;
        using Allocator         = boost::interprocess::allocator<Element, ShmSegmentManager>;
        using Container         = boost::circular_buffer<Element, Allocator>;
        using Mutex             = boost::interprocess::interprocess_mutex;
        using ConditionVariable = boost::interprocess::interprocess_condition;
        using ElementCount      = typename Container::size_type;

        explicit CircularQueue(ElementCount capacity, ShmManager& shm) :
            m_allocator{shm.get_segment_manager()},
            m_data(capacity, m_allocator), // use () or else very nasty compile time errors occur
            m_mutex{},
            m_cv{}
        {}

        /*
         * Disable copy and move semantics.
        */
        CircularQueue(const CircularQueue&) = delete;
        CircularQueue(CircularQueue&&) = delete;
        CircularQueue& operator=(const CircularQueue&) = delete;
        CircularQueue& operator=(CircularQueue&&) = delete;

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
        ElementCount put(Element&& value)
        {
            auto lk{makeLock()};
            m_data.push_back(std::move(value));
            m_cv.notify_all();
            return m_data.size();
        }

        /*
         * Puts an lvalue element onto the queue, i.e. the element gets copied.
        */
        ElementCount put(const Element& value)
        {
            auto lk{makeLock()};
            m_data.push_back(value);
            m_cv.notify_all();
            return m_data.size();
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
        bool waitUntilAndPop(Element& value, const boost::posix_time::ptime& absoluteTime)
        {
            auto lk{makeLock()};
            const auto isDataAvailable = m_cv.timed_wait(lk, absoluteTime,
                                                         [this]{return !m_data.empty();});

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
        boost::optional<Element> waitUntilAndPop(const boost::posix_time::ptime& absoluteTime)
        {
            auto lk{makeLock()};
            const auto isDataAvailable = m_cv.timed_wait(lk, absoluteTime,
                                                         [this]{return !m_data.empty();});
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
        bool waitForAndPop(Element& value, const boost::posix_time::milliseconds& dur)
        {
            // Note: universal time must be used, or else the code execution results in
            //       the code hanging and never timeout.
            // Also see the warning messages from here: https://www.boost.org/doc/libs/1_63_0/doc/html/interprocess/synchronization_mechanisms.html#interprocess.synchronization_mechanisms.conditions
            return waitUntilAndPop(value, boost::posix_time::microsec_clock::universal_time() + dur);
        }

        boost::optional<Element> waitForAndPop(const boost::posix_time::milliseconds& dur)
        {
            return waitUntilAndPop(boost::posix_time::microsec_clock::universal_time() + dur);
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
        ElementCount getSize() const
        {
            auto lk{makeLock()};
            return m_data.size();
        }

        ElementCount getCapacity() const
        {
            return m_data.capacity();
        }

    private:
        boost::interprocess::scoped_lock<Mutex> makeLock() const
        {
            return boost::interprocess::scoped_lock<Mutex>{m_mutex};
        }

    private:
        Allocator                  m_allocator;
        Container                  m_data;
        mutable Mutex              m_mutex;
        mutable ConditionVariable  m_cv;
    };

}}}


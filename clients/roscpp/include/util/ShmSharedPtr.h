/***************************************************************************************************
 * Copyright Five AI 2021.
 * All rights reserved.
 ***************************************************************************************************/

#pragma once

#include <boost/interprocess/smart_ptr/shared_ptr.hpp>
#include <boost/interprocess/managed_shared_memory.hpp>

#include <type_traits>

namespace fiveai { namespace shm
{
    namespace __shared_ptr
    {
        // Do not use boost::interprocess::managed_shared_ptr to construct boost::interprocess::shared_ptr type,
        // or else partial specialisation won't be able to deduce the embedded type T
        using VoidAllocator = typename boost::interprocess::managed_shared_memory::allocator<void>::type;

        // Need to remove the constness to allow SharedPtr<T> -> SharedPtr<const T> implicit conversions
        template <typename T>
        using Deleter = typename boost::interprocess::managed_shared_memory::deleter<typename std::remove_cv<T>::type>::type;
    }

    template <typename T>
    using SharedPtr = boost::interprocess::shared_ptr
    <
        T,
        __shared_ptr::VoidAllocator,
        __shared_ptr::Deleter<T>
    >;
}}


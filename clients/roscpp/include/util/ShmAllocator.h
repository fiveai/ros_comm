/***************************************************************************************************
 * Copyright Five AI 2021.
 * All rights reserved.
 ***************************************************************************************************/

#pragma once

#include <boost/interprocess/allocators/allocator.hpp>
#include <boost/interprocess/managed_shared_memory.hpp>

namespace fiveai { namespace shm
{
    /// A STL compatible allocator able to reserve/release shared memory
    /// of the type specified by template @param T
    template <typename T>
    using Allocator = boost::interprocess::allocator<T, boost::interprocess::managed_shared_memory::segment_manager>;

    using StringAllocator = Allocator<char>;
}}

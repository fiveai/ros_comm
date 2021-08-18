/***************************************************************************************************
 * Copyright Five AI Limited 2021
 * All rights reserved

 * This file is released under a Creative Commons Attribution-NonCommercial-ShareAlike 4.0
 * International License. Please refer to the README.md file supplied with this repository.
 ***************************************************************************************************/

#pragma once

#include <boost/interprocess/allocators/allocator.hpp>
#include <boost/interprocess/managed_shared_memory.hpp>

namespace ros { namespace shm
{
    /// A STL compatible allocator able to reserve/release shared memory
    /// of the type specified by template @param T
    template <typename T>
    using Allocator = boost::interprocess::allocator<T, boost::interprocess::managed_shared_memory::segment_manager>;

    using StringAllocator = Allocator<char>;
}}

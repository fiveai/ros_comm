/***************************************************************************************************
 * Copyright Five AI 2021.
 * All rights reserved.
 ***************************************************************************************************/

#pragma once

#include "ShmAllocator.h"

#include <boost/interprocess/containers/vector.hpp>

namespace ros { namespace shm
{
    template <typename T, typename A = shm::Allocator<T>>
    using Vector = boost::interprocess::vector<T, A>;
}}

/***************************************************************************************************
 * Copyright Five AI Limited 2021
 * All rights reserved

 * This file is released under a Creative Commons Attribution-NonCommercial-ShareAlike 4.0
 * International License. Please refer to the README.md file supplied with this repository.
 ***************************************************************************************************/

#pragma once

#include "ShmAllocator.h"

#include <boost/interprocess/containers/vector.hpp>

namespace ros { namespace shm
{
    template <typename T, typename A = shm::Allocator<T>>
    using Vector = boost::interprocess::vector<T, A>;
}}

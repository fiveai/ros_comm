/***************************************************************************************************
 * Copyright Five AI Limited 2021
 * All rights reserved

 * This file is released under a Creative Commons Attribution-NonCommercial-ShareAlike 4.0
 * International License. Please refer to the README.md file supplied with this repository.
 ***************************************************************************************************/

#pragma once

#include "ShmAllocator.h"

#include <boost/interprocess/containers/string.hpp>

#include <string> // for std::char_traits

namespace ros { namespace shm
{
    template
    <
        typename Char = char,
        typename A = shm::Allocator<Char>
    >
    using _String = boost::interprocess::basic_string<Char, std::char_traits<Char>, A>;

    using String = _String<>;

    inline
    String makeString(const std::string& str, boost::interprocess::managed_shared_memory& shmManager)
    {
        StringAllocator allocator{shmManager.get_segment_manager()};
        return {str.c_str(), allocator};
    }
}}


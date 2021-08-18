/***************************************************************************************************
 * Copyright Five AI Limited 2021
 * All rights reserved

 * This file is released under a Creative Commons Attribution-NonCommercial-ShareAlike 4.0
 * International License. Please refer to the README.md file supplied with this repository.
 ***************************************************************************************************/
#pragma once

#include "threading/Attributes.h"

#include <iosfwd>

namespace ros { namespace threading
{
    std::ostream& operator<<(std::ostream& os, const Nice& nice);
    template <typename Policy>
    std::ostream& operator<<(std::ostream& os, const StaticPriority<Policy>& staticPriority);
    std::ostream& operator<<(std::ostream& os, const CpuAffinity& affinity);
    std::ostream& operator<<(std::ostream& os, const CpuAffinitySet& affinities);
    std::ostream& operator<<(std::ostream& os, const StackSize& stackSize);

    std::ostream& operator<<(std::ostream& os, const StandardAttributes& attributes);
    std::ostream& operator<<(std::ostream& os, const RealTimeAttributes& attributes);


    std::string toString(const Nice& nice);
    std::string toString(const CpuAffinity& affinity);
    std::string toString(const CpuAffinitySet& affinities);
    std::string toString(const StackSize& stackSize);
    std::string toString(const StandardAttributes& attributes);
    std::string toString(const RealTimeAttributes& attributes);
}}

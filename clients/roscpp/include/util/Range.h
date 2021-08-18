/***************************************************************************************************
 * Copyright Five AI Limited 2021
 * All rights reserved

 * This file is released under a Creative Commons Attribution-NonCommercial-ShareAlike 4.0
 * International License. Please refer to the README.md file supplied with this repository.
 ***************************************************************************************************/

#pragma once

namespace ros { namespace util
{
    template <typename T>
    struct Range
    {
        bool contains(const T& val) const
        {
            return low <= val && val <= high;
        }

        bool containsStrict(const T& val) const
        {
            return low < val && val < high;
        }

        bool containsStrictLow(const T& val) const
        {
            return low < val && val <= high;
        }

        bool containsStrictHigh(const T& val) const
        {
            return low <= val && val < high;
        }

        T low;
        T high;
    };

}}

/***************************************************************************************************
 * Copyright Five AI 2021.
 * All rights reserved.
 ***************************************************************************************************/

#pragma once

namespace fiveai { namespace platform { namespace util
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

}}}

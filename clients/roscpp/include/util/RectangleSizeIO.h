/***************************************************************************************************
 * Copyright Five AI 2021.
 * All rights reserved.
 ***************************************************************************************************/

#pragma once

#include "util/RectangleSize.hpp"

#include <ostream>
#include <sstream>
#include <string>

namespace fiveai { namespace util
{
    template <typename T>
    inline std::ostream& operator<<(std::ostream& os, const RectangleSize<T>& r)
    {
        os << r.getWidth() << "x" << r.getHeight();
        return os;
    }

    template <typename T>
    inline std::string toString(const RectangleSize<T>& r)
    {
        std::ostringstream oss;
        oss << r;
        return oss.str();
    }
}}

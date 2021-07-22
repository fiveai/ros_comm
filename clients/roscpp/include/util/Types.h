/***************************************************************************************************
 * Copyright Five AI 2021.
 * All rights reserved.
 ***************************************************************************************************/

#pragma once

#include "util/RectangleSize.h"

#include <cstdint>

namespace ros { namespace util
{
    using ElementCount = std::uint64_t;
    using ByteCount = std::uint64_t;
    using PixelCount = std::uint64_t;

    using SizePixels = RectangleSize<PixelCount>;
}}

#pragma once

#include "util/RectangleSize.h"

#include <cstdint>

namespace fiveai { namespace platform { namespace util
{
    using ElementCount = std::uint64_t;
    using ByteCount = std::uint64_t;
    using PixelCount = std::uint64_t;

    // Note, SizeBytes does not make sense!
    using SizeElements = RectangleSize<ElementCount>;
    using SizePixels = RectangleSize<PixelCount>;
}}}

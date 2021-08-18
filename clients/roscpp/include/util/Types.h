/***************************************************************************************************
 * Copyright Five AI Limited 2021
 * All rights reserved

 * This file is released under a Creative Commons Attribution-NonCommercial-ShareAlike 4.0
 * International License. Please refer to the README.md file supplied with this repository.
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

/***************************************************************************************************
 * Copyright Five AI Limited 2021
 * All rights reserved

 * This file is released under a Creative Commons Attribution-NonCommercial-ShareAlike 4.0
 * International License. Please refer to the README.md file supplied with this repository.
 ***************************************************************************************************/
#pragma once

#include <boost/thread/thread_only.hpp>

namespace ros { namespace threading
{
    using OsId = boost::thread::id;
}}

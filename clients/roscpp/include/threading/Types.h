/***************************************************************************************************
 * Copyright Five AI 2021.
 * All rights reserved.
 ***************************************************************************************************/
#pragma once

#include <boost/thread/thread_only.hpp>

namespace ros { namespace threading
{
    using OsId = boost::thread::id;
}}

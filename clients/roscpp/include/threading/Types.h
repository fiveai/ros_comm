/***************************************************************************************************
 * Copyright Five AI 2017.
 * All rights reserved.
 ***************************************************************************************************/
#pragma once

#include <boost/thread/thread_only.hpp>

namespace fiveai { namespace platform { namespace threading
{
    using OsId = boost::thread::id;
}}}

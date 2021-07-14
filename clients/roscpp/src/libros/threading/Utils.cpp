/***************************************************************************************************
 * Copyright Five AI 2021.
 * All rights reserved.
 ***************************************************************************************************/

#include "threading/Utils.h"

#include "ros/assert.h"

#include <pthread.h>

namespace fiveai { namespace threading
{

int baptizeThisThread(const std::string& name)
{
    const auto id = pthread_self();
    ROS_ASSERT_MSG(id != 0, "Invalid thread id");

    // Name length is restricted to 16 characters, including the terminating null byte ('\0')
    const std::string truncatedName = name.substr(0, 15);

    return ::pthread_setname_np(id, truncatedName.c_str());
}

}}

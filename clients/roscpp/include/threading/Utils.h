/***************************************************************************************************
 * Copyright Five AI 2021.
 * All rights reserved.
 ***************************************************************************************************/

#pragma once

#include <string>

#include <future>
#include <functional>

namespace ros { namespace threading
{
    /**
     * Assigns @param name to the calling thread.
     * Returns whatever pthread_setname_np would return
     */
    int baptizeThisThread(const std::string& name);

    template <typename Ret>
    inline void executeAndSetBarrier(std::promise<Ret>& barrier, std::function<Ret()> cmd)
    {
        Ret ret = cmd();
        barrier.set_value(ret);
    }

    template <>
    inline void executeAndSetBarrier<void>(std::promise<void>& barrier, std::function<void()> cmd)
    {
        cmd();
        barrier.set_value();
    }

}}

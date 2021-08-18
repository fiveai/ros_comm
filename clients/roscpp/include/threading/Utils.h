/***************************************************************************************************
 * Copyright Five AI Limited 2021
 * All rights reserved

 * This file is released under a Creative Commons Attribution-NonCommercial-ShareAlike 4.0
 * International License. Please refer to the README.md file supplied with this repository.
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

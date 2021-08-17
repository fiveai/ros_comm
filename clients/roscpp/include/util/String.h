/***************************************************************************************************
 * Copyright Five AI Limited 2021
 * All rights reserved

 * This file is released under a Creative Commons Attribution-NonCommercial-ShareAlike 4.0
 * International License. Please refer to the README.md file supplied with this repository.
 ***************************************************************************************************/
#pragma once

#include <cstdint>
#include <string>
#include <algorithm>
#include <functional>

namespace ros { namespace util
{
    template <typename C>
    std::string foldToString(const C& container,
                             std::function<std::string(const typename C::value_type&)> converter = [](const typename C::value_type& elem){return elem;},
                             const char separator = '|',
                             const std::string& emptyContainerMessage = "Empty set")
    {
        if (container.empty())
        {
            return emptyContainerMessage;
        }

        auto folder = [&converter, separator](std::string s, const typename C::value_type& elem)
        {
            auto tmp = std::move(s) + separator + converter(elem);
            return tmp;
        };

        std::string str{converter(*container.begin())};
        return std::accumulate(std::next(container.begin()), container.end(), str, folder);
    }

    // http://man7.org/linux/man-pages/man3/shm_open.3.html
    std::string portableShmName(const std::string& str);
    std::string portableShmQueueName(const std::string& str);
}}

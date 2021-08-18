/***************************************************************************************************
 * Copyright Five AI Limited 2021
 * All rights reserved

 * This file is released under a Creative Commons Attribution-NonCommercial-ShareAlike 4.0
 * International License. Please refer to the README.md file supplied with this repository.
 ***************************************************************************************************/

#include "benchmark/Util.h"

#include <ros/console.h>

#include <stdexcept>

bool isTransportSupported(const std::string& transport)
{
    return transport == "shm" ||
           transport == "tcp" ||
           transport == "udp" ||
           transport == "tzc";
}

void throwIfTransportNotSupported(const std::string& transport)
{
    if (! isTransportSupported(transport))
    {
        const auto msg = std::string("Unknown transport protocol ") + transport;
        ROS_ERROR_STREAM(msg);
        throw std::logic_error{msg};
    }
}

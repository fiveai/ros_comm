/***************************************************************************************************
 * Copyright Five AI 2021.
 * All rights reserved.
 ***************************************************************************************************/

#pragma once

#include <ros/node_handle.h>

#include <string>
#include <stdexcept>

bool isTransportSupported(const std::string& transport);
void throwIfTransportNotSupported(const std::string& transport);

// Returns the same as NodeHandle::getParam but rather than returning false
// it throws std::logic_error should the parameter not be found.
template <typename T>
inline
T getParam(ros::NodeHandle& nh, const std::string& key)
{
    T val;
    if (! nh.getParam(key, val))
    {
        throw std::logic_error{"Param identified by key " + key + " not found"};
    }
    return val;
}

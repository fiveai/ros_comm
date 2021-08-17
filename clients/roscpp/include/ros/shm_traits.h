/***************************************************************************************************
 * Copyright Five AI Limited 2021
 * All rights reserved

 * This file is released under a Creative Commons Attribution-NonCommercial-ShareAlike 4.0
 * International License. Please refer to the README.md file supplied with this repository.
 ***************************************************************************************************/
 
#pragma once

#include "msgs/ShmImage.h"

namespace ros
{
    template <typename M>
    struct IsShm
    {
      constexpr static bool value = std::is_same<ros::lot_msgs::SharedPtrShmImage, M>::value        ||
                                    std::is_same<ros::lot_msgs::SharedPtrConstShmImage, M>::value   
        ;
    };
}

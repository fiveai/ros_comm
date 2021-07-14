/***************************************************************************************************
 * Copyright Five AI 2021.
 * All rights reserved.
 ***************************************************************************************************/

#pragma once

#include "msgs/ShmImage.h"

namespace ros
{
    template <typename M>
    struct IsShm
    {
      constexpr static bool value = std::is_same<fiveai::std_msgs::shm::SharedPtrShmImage, M>::value        ||
                                    std::is_same<fiveai::std_msgs::shm::SharedPtrConstShmImage, M>::value   
        ;
    };
}

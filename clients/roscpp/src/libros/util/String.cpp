/***************************************************************************************************
 * Copyright Five AI 2021.
 * All rights reserved.
 ***************************************************************************************************/

#include "util/String.h"

#include <boost/algorithm/string/replace.hpp>
#include <boost/algorithm/string/trim.hpp>

namespace fiveai { namespace platform { namespace util
{

std::string portableShmName(const std::string& s)
{
    if (s.empty())
    {
        return "/shared_memory_object";
    }

    using namespace boost::algorithm;

    std::string copy{s};

    trim_left_if(copy, is_any_of("/"));
    replace_all(copy, "/", "_");
    copy.insert(copy.begin(), '/');

    // truncate to 255 characters including the NULL character
    constexpr std::size_t MAX_LENGTH = 254;
    if (copy.size() >= MAX_LENGTH)
    {
        return {copy.begin(), std::next(copy.begin(), MAX_LENGTH)};
    }

    return copy;
}

std::string portableShmQueueName(const std::string& s)
{
     if (s.empty())
     {
         throw std::logic_error{"Empty string is not a valid string for identifying objects in shared memory"};
     }

     auto name = "Q_" + s;
     boost::algorithm::replace_all(name, "/", "");
     return name;
}

}}}

/***************************************************************************************************
 * Copyright Five AI 2017.
 * All rights reserved.
 ***************************************************************************************************/
#pragma once

#include <cstdint>
#include <string>
#include <algorithm>
#include <functional>
#include <numeric>

namespace fiveai { namespace platform { namespace util
{
    namespace internal
    {
        constexpr const char* baseNameImpl(const char* p, std::size_t offset)
        {
            return (p[offset] == 0) ? p :
                                      ((p[offset] == '/') ? baseNameImpl(p + offset + 1, 0) :
                                                            baseNameImpl(p, offset + 1));
        }

        constexpr std::uint32_t SEED = 5381;
        constexpr std::uint32_t hashImpl(const char *s, std::size_t offset, std::uint32_t hash)
        {
            return (s[offset] == 0) ? hash :
                                      hashImpl(s, offset + 1, (hash * 33) ^ s[offset]);
        }
    }

    /*
     * Remove all parts of file path except the actual file name at compile time.
     * Example: after applying baseName to 'lib/foo/bar.c' the returned value is 'bar.c'
    */
    constexpr const char* baseName(const char* filePath)
    {
        return internal::baseNameImpl(filePath, 0);
    }

    /*
     * Compile time function to compute the hash of a string @param s (Dan Bernstein).
     * Returns the hash represented as an integer.
     *
     * TODO: get rid of this function and use Boost.Hana when it becomes available (> 1.61)
     */
    constexpr std::uint32_t hash(const char *s)
    {
        return internal::hashImpl(s, 0, internal::SEED);
    }

    /*
     * TODO: explain
    */
    template <typename T>
    inline
    std::string toString(T val)
    {
        return std::to_string(val);
    }

    inline
    const std::string& toString(const std::string& val)
    {
        return val;
    }

    inline
    std::string toString(std::string&& val)
    {
        return std::move(val);
    }

    inline
    std::string toString(const char* val)
    {
        return {val};
    }


    template <typename C>
    std::string foldToString(
        const C& container,
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
}}}

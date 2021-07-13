/***************************************************************************************************
 * Copyright Five AI 2021.
 * All rights reserved.
 ***************************************************************************************************/
#pragma once

#include <iosfwd>
#include <stdexcept>
#include <system_error>
#include <string>
#include <functional>

namespace ros { namespace util
{
    /*
     * Writes the error message for exception ex to output stream os, followed by a | symbol. If the exception is
     * nested, the message for each nested exception is written, separated by | symbols.
     */
    void printExceptionInfo(const std::exception& ex, std::ostream& os) noexcept;

    /*
     * Calls printExceptionInfo for the currently handled exception and output stream os.
     */
    void printCurrentExceptionInfo(std::ostream& os) noexcept;

    /*
     * Calls printCurrentExceptionInfo(os) and returns the string written to os.
     */
    std::string currentExceptionInfo() noexcept;

    /**
     * Calls @fn f and throws @exception std::system_error in the event
     * that the return code of the supplied function is less than 0.
     *
     * The exception error message can be customised/extended by means @param msg.
     * The @param msg is supplied as a lambda function returning a @keyword std::string
     * the reason being efficiency, the std::string error message is constructed only
     * an exception needs to be thrown.
    */
    template <typename F, typename... Args>
    inline int tryInvoke(F f, std::function<std::string()> msg, Args&&... args)
    {
        auto ret = f(std::forward<Args>(args)...);

        if (ret < 0)
        {
            throw std::system_error(errno, std::system_category(), msg());
        }

        return ret;
    }

}}

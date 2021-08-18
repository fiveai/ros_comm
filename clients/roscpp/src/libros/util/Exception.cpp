/***************************************************************************************************
 * Copyright Five AI Limited 2021
 * All rights reserved

 * This file is released under a Creative Commons Attribution-NonCommercial-ShareAlike 4.0
 * International License. Please refer to the README.md file supplied with this repository.
 ***************************************************************************************************/

#include "util/Exception.h"

#include <exception>
#include <sstream>

namespace ros { namespace util
{
    void printExceptionInfo(const std::exception& ex, std::ostream& os) noexcept
    {
        os << ex.what() << "|";
        try
        {
            std::rethrow_if_nested(ex);
        }
        catch(const std::exception& e)
        {
            printExceptionInfo(e, os);
        }
        catch(...)
        {
            os << "Unknown exception in " << __PRETTY_FUNCTION__;
        }
    }

    void printCurrentExceptionInfo(std::ostream& os) noexcept
    {
        auto eptr = std::current_exception();

        bool canPrintExceptionInfo = static_cast<bool>(eptr);

        try
        {
           if (eptr)
           {
               std::rethrow_exception(eptr);
           }
        }
        catch(const std::exception& e)
        {
            printExceptionInfo(e, os);
        }
        catch(...)
        {
            canPrintExceptionInfo = false;
        }

        if (! canPrintExceptionInfo)
        {
            os << "Unknown exception in " << __PRETTY_FUNCTION__ << std::endl;
        }
    }

    std::string currentExceptionInfo() noexcept
    {
        std::ostringstream oss;
        printCurrentExceptionInfo(oss);
        return oss.str();
    }
}}

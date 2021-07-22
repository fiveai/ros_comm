/***************************************************************************************************
 * Copyright Five AI 2021.
 * All rights reserved.
 ***************************************************************************************************/


#include "threading/AttributesIO.h"
#include "util/String.h"

#include <sstream>
#include <string>
#include <algorithm>

namespace ros { namespace threading
{
    std::ostream& operator<<(std::ostream& os, const Nice& nice)
    {
        os << nice.getValue();
        return os;
    }

    template <typename Policy>
    std::ostream& operator<<(std::ostream& os, const StaticPriority<Policy>& staticPriority)
    {
        using scheduling_policy::toString;
        os << "(" << staticPriority.getValue() << "," << toString(staticPriority.getPolicy()) << ")";
        return os;
    }

    std::ostream& operator<<(std::ostream& os, const CpuAffinity& affinity)
    {
        os << affinity.getValue();
        return os;
    }

    std::ostream& operator<<(std::ostream& os, const CpuAffinitySet& affinities)
    {
        auto toString = [](CpuAffinity affinity)
        {
            std::ostringstream oss;
            oss << affinity;
            return oss.str();
        };
        os << util::foldToString(affinities, toString, ',', "OS default");
        return os;
    }

    std::ostream& operator<<(std::ostream& os, const StackSize& stackSize)
    {
        os << stackSize.getValue();
        return os;
    }

    std::ostream& operator<<(std::ostream& os, const Attributes& attributes)
    {
        using scheduling_policy::toString;

        os << "name="           << attributes.getName()             << "|"
           << "cpuAffinities="  << attributes.getCpuAffinities()    << "|"
           << "stackSizeBytes=" << attributes.getStackSize();

        return os;
    }

    std::ostream& operator<<(std::ostream& os, const StandardAttributes& attributes)
    {
        using scheduling_policy::toString;

        os << static_cast<const Attributes&>(attributes)    << "|"
           << "policy=" << toString(attributes.getPolicy()) << "|"
           << "nice="   << attributes.getNice();

        return os;
    }

    std::ostream& operator<<(std::ostream& os, const RealTimeAttributes& attributes)
    {
        using scheduling_policy::toString;

        os << static_cast<const Attributes&>(attributes)            << "|"
           << "policy="         << toString(attributes.getPolicy()) << "|"
           << "staticPriority=" << attributes.getStaticPriority();

        return os;
    }

    template <typename T>
    std::string str(const T& val)
    {
        std::ostringstream oss;
        oss << val;
        return oss.str();
    }

    std::string toString(const Nice& nice)
    {
        return str(nice);
    }

    std::string toString(const CpuAffinity& affinity)
    {
        return str(affinity);
    }

    std::string toString(const CpuAffinitySet& affinities)
    {
        return str(affinities);
    }

    std::string toString(const StackSize& stackSize)
    {
        return str(stackSize);
    }

    std::string toString(const StandardAttributes& attributes)
    {
        return str(attributes);
    }

    std::string toString(const RealTimeAttributes& attributes)
    {
        return str(attributes);
    }
}}

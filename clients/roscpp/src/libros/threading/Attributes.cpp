/***************************************************************************************************
 * Copyright Five AI 2021.
 * All rights reserved.
 ***************************************************************************************************/


#include "threading/Attributes.h"

#include "util/Exception.h"
#include "util/Macros.h"

#include <boost/type_index.hpp>
#include <boost/container/flat_map.hpp>
#include <boost/range/algorithm/count_if.hpp>
#include <boost/thread/thread.hpp>

#include <stdexcept>
#include <thread>
#include <algorithm>

#include <sys/resource.h>

namespace ros { namespace threading
{
    namespace scheduling_policy
    {
        namespace
        {
            const std::string STANDARD_POLICIES_STRINGS[] =
            {
               FIVEAI_TO_STRINGIZE_ENUM(FIVEAI_SEQ_STANDARD_SCHEDULING_POLICIES)
            };

            const std::string REAL_TIME_POLICIES_STRINGS[] =
            {
               FIVEAI_TO_STRINGIZE_ENUM(FIVEAI_SEQ_REAL_TIME_SCHEDULING_POLICIES)
            };

            const boost::container::flat_map<Standard, int> STANDARD_POLICIES_VALUES =
            {{
                FIVEAI_GENERATE_BRACED_ENUM_VALUE_PAIRS(
                    FIVEAI_SEQ_STANDARD_SCHEDULING_POLICIES,
                    scheduling_policy::Standard
                )
            }};

            const boost::container::flat_map<RealTime, int> REAL_TIME_POLICIES_VALUES =
            {{
                FIVEAI_GENERATE_BRACED_ENUM_VALUE_PAIRS(
                    FIVEAI_SEQ_REAL_TIME_SCHEDULING_POLICIES,
                    scheduling_policy::RealTime
                )
            }};
        }

        std::string toString(Standard policy)
        {
            return STANDARD_POLICIES_STRINGS[static_cast<int>(policy)];
        }

        std::string toString(RealTime policy)
        {
            return REAL_TIME_POLICIES_STRINGS[static_cast<int>(policy)];
        }

        int valueOf(Standard policy)
        {
            return STANDARD_POLICIES_VALUES.at(policy);
        }

        int valueOf(RealTime policy)
        {
            return REAL_TIME_POLICIES_VALUES.at(policy);
        }
    }

Attributes::Attributes(const std::string& name,
    CpuAffinitySet cpuAffinities,
    StackSize stackSize
) :
    m_cpuAffinities{cpuAffinities},
    m_name{name},
    m_stackSize{stackSize}
{
}

bool Attributes::areCpuAfinitiesValid() const
{
    auto predicate = [](const CpuAffinity& affinity){return ! affinity.isValid();};
    return boost::count_if(getCpuAffinities(), predicate) == 0;
}

StandardAttributes::StandardAttributes(
    const std::string& name,
    const CpuAffinitySet& cpuAffinities,
    StackSize stackSize,
    SchedulingPolicy policy,
    Nice nice
) :
    Attributes{name, cpuAffinities, stackSize},
    m_policy{policy},
    m_nice{nice}
{
    if (! isValid())
    {
        throw std::logic_error{"Invalid attributes for standard thread " + name};
    }
}

bool StandardAttributes::isValid() const
{
    return areCpuAfinitiesValid() &&
           getNice().isValid();
}

std::string StandardAttributes::getPolicyName() const
{
    return toString(getPolicy());
}

RealTimeAttributes::RealTimeAttributes(
    const std::string& name,
    const CpuAffinitySet& cpuAffinities,
    StackSize stackSize,
    StaticPriority staticPriority
) :
    Attributes{name, cpuAffinities, stackSize},
    m_staticPriority{staticPriority}
{
    if (! isValid())
    {
        throw std::logic_error{"Invalid attributes for real time thread " + name};
    }
}

bool RealTimeAttributes::isValid() const
{
    return areCpuAfinitiesValid() &&
           getStaticPriority().isValid();
}

std::string RealTimeAttributes::getPolicyName() const
{
    return toString(getPolicy());
}

util::Range<Nice> Nice::getRange()
{
    return {Nice{20}, Nice{-19}};
}

Nice Nice::getDefault()
{
    return Nice{0};
}

template <typename Policy>
util::Range<StaticPriority<Policy>> StaticPriority<Policy>::getRange(Policy policy)
{
    using util::tryInvoke;
    using scheduling_policy::valueOf;

    auto msg = []
    {
        return std::string{"Could not retrieve max/min thread static priority"};
    };

    const StaticPriority<Policy> minPriority {policy, tryInvoke(sched_get_priority_min, msg, valueOf(policy) )};
    const StaticPriority<Policy> maxPriority {policy, tryInvoke(sched_get_priority_max, msg, valueOf(policy) )};
    return {minPriority, maxPriority};
}

util::Range<CpuAffinity> CpuAffinity::getRange()
{
    const auto coreCount = std::thread::hardware_concurrency();
    if (coreCount == 0)
    {
        throw std::logic_error{"Could not detect the number of cores!"};
    }
    return {CpuAffinity{0}, CpuAffinity{coreCount -1}};
}

std::vector<CpuAffinity> CpuAffinity::getDefault()
{
    /// Empty set means that this property is not set by the @class Thread.
    /// This is equivalent to letting the OS decide the affinity(ies) of the thread.
    return {};
}

util::Range<StackSize> StackSize::getRange()
{
    rlimit stackLimit;
    auto msg = []{return std::string{"Could not retrieve stack limit"};};

    util::tryInvoke(getrlimit, msg, RLIMIT_STACK, &stackLimit);

    return {StackSize{0}, StackSize{stackLimit.rlim_max}};
}

StackSize StackSize::getDefault()
{
    // Note, next call returns the calling thread's stack size!
    return StackSize{boost::thread_attributes{}.get_stack_size()};
}

}}

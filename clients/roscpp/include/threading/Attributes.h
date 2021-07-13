/***************************************************************************************************
 * Copyright Five AI 2021.
 * All rights reserved.
 ***************************************************************************************************/
#pragma once

#include "threading/defs/SchedulingPolicy.def"
#include "threading/Property.h"
#include "util/Range.h"
#include "util/Types.h"

#include <boost/preprocessor/seq/enum.hpp>
#include <boost/preprocessor/seq/size.hpp>

#include <string>
#include <vector>

#include <sched.h>

namespace ros { namespace threading
{
    namespace scheduling_policy
    {
        enum class Standard
        {
            BOOST_PP_SEQ_ENUM(LOT_SEQ_STANDARD_SCHEDULING_POLICIES)
        };

        enum class RealTime
        {
            BOOST_PP_SEQ_ENUM(LOT_SEQ_REAL_TIME_SCHEDULING_POLICIES)
        };

        template <Standard policy> constexpr int valueOf();
        template <> constexpr int valueOf<Standard::OTHER>() {return SCHED_OTHER;}
        template <> constexpr int valueOf<Standard::IDLE>()  {return SCHED_IDLE;}
        template <> constexpr int valueOf<Standard::BATCH>() {return SCHED_BATCH;}
        int valueOf(Standard policy);
        std::string toString(Standard policy);

        template <RealTime policy> constexpr int valueOf();
        template <> constexpr int valueOf<RealTime::FIFO>()        {return SCHED_FIFO;}
        template <> constexpr int valueOf<RealTime::ROUND_ROBIN>() {return SCHED_RR;}
        int valueOf(RealTime policy);
        std::string toString(RealTime policy);
    }

    /**
     * From http://man7.org/linux/man-pages/man7/sched.7.html
     *      The nice value is an attribute that can be used to influence the CPU
     *      scheduler to favor or disfavor a process in scheduling decisions.
     *
     *      On Linux, the nice value is a per-thread attribute: different threads in
     *      the same process may have different nice values.
     *
     *      On modern Linux, the range is -20 (high priority) to +19 (low priority).
    */
    struct Nice : public Property<int, Nice>
    {
        using Base = Property<int, Nice>;
        using Base::Base;

        /*
         * The highest nice priority is -20 and lowest 19, hence we
         * need to reverse the operators >, += and -=.
        */
        bool operator<(const Nice& rhs) const
        {
            return getValue() > rhs.getValue();
        }

        Nice& operator+=(const Nice& rhs)
        {
            trySetValue(getValue() - rhs.getValue());
            return *this;
        }

        Nice& operator-=(const Nice& rhs)
        {
            trySetValue(getValue() + rhs.getValue());
            return *this;
        }

        bool isValid() const
        {
            return getRange().contains(*this);
        }

        static util::Range<Nice> getRange();
        static Nice getDefault();
    };

    template <typename Policy>
    struct StaticPriority : public Property<int, StaticPriority<Policy>>
    {
        using Base = Property<int, StaticPriority>;
        using SchedulingPolicy = Policy;

        StaticPriority(SchedulingPolicy policy, int value) :
            Base{value},
            m_policy{policy}
        {}

        SchedulingPolicy getPolicy() const
        {
            return m_policy;
        }

        bool isValid() const
        {
            return getRange(m_policy).contains(*this);
        }

        static util::Range<StaticPriority<SchedulingPolicy>> getRange(SchedulingPolicy policy);

    private:
        Policy m_policy;
    };

    // Factory function which helps deduce the type of the policy automatically
    template <typename Policy>
    inline StaticPriority<Policy> makeStaticPriority(Policy policy, int value)
    {
        return StaticPriority<Policy>{policy, value};
    }

    struct CpuAffinity : public Property<unsigned int, CpuAffinity>
    {
        using Base = Property<unsigned int, CpuAffinity>;
        using Base::Base;

        bool isValid() const
        {
            return getRange().contains(*this);
        }

        static util::Range<CpuAffinity> getRange();
        static std::vector<CpuAffinity> getDefault();
    };

    /// A collection of CPU ids which the OS uses to run the thread
    using CpuAffinitySet = std::vector<CpuAffinity>;

    /**
     * Specifies the size of the thread's stack in bytes.
     */
    struct StackSize : public Property<util::ByteCount, StackSize>
    {
        using Base = Property<std::size_t, StackSize>;
        using Base::Base;

        bool isValid() const
        {
            return getRange().contains(*this);
        }

        static util::Range<StackSize> getRange();
        static StackSize getDefault();
    };

    class Attributes
    {
    public:
        Attributes(const std::string& name,
            CpuAffinitySet cpuAffinities,
            StackSize stackSize
        );

        const CpuAffinitySet& getCpuAffinities() const
        {
            return m_cpuAffinities;
        }

        bool areCpuAfinitiesValid() const;

        StackSize getStackSize() const
        {
            return m_stackSize;
        }

        const std::string& getName() const
        {
            return m_name;
        }

        void setName(const std::string& name)
        {
            m_name = name;
        }

        void addNameSuffix(const std::string& suffix)
        {
            m_name += suffix;
        }

        void addNamePrefix(const std::string& prefix)
        {
            m_name = prefix + m_name;
        }

    protected:
        // forbid destroying derived classes through pointer to Attributes
        ~Attributes() noexcept = default;

    private:
        CpuAffinitySet m_cpuAffinities;
        // Denotes the name of this thread
        std::string m_name;
        StackSize   m_stackSize;
    };

    template <typename A>
    inline
    A addNameSuffixCopy(const A& attributes, const std::string& suffix)
    {
        A ret{attributes};
        ret.addNameSuffix(suffix);
        return ret;
    }

    template <typename A>
    inline
    A addNamePrefixCopy(const A& attributes, const std::string& prefix)
    {
        A ret{attributes};
        ret.addNamePrefix(prefix);
        return ret;
    }

    /**
     * @class RealTimeAttributes can be passed to @class Thread to create
     * a thread that the OS will schedule using @enum RealTime policies. The
     * class is intended to be employed when a finer control over how runnable
     * threads are chosen for execution is necessary.
     *
     * @param name - specifies a string to identify the thread instance
     *               in the OS
     * @param cpuAffinity - on multi-core machines specifies the core on which
     *                      the thread will be run
    */
    class RealTimeAttributes : public Attributes
    {
    public:
        using SchedulingPolicy = scheduling_policy::RealTime;
        using StaticPriority = threading::StaticPriority<SchedulingPolicy>;

        RealTimeAttributes(
            const std::string& name = "rt-th",
            const CpuAffinitySet& cpuAffinities = CpuAffinity::getDefault(),
            StackSize stackSize = StackSize::getDefault(),
            StaticPriority staticPriority = StaticPriority::getRange(scheduling_policy::RealTime::FIFO).low);

        StaticPriority getStaticPriority() const
        {
            return m_staticPriority;
        }

        SchedulingPolicy getPolicy() const
        {
            return m_staticPriority.getPolicy();
        }

        std::string getPolicyName() const;

        bool isValid() const;

    private:
        StaticPriority m_staticPriority;
    };

    /**
     * @class StandardAttributes can be passed to @class Thread to create
     * a thread that the OS will schedule using @enum StandardTime policies.
     * The class is intended to be employed when the default/normal scheduling
     * policy suffices.
     */
    class StandardAttributes : public Attributes
    {
    public:
        using SchedulingPolicy = scheduling_policy::Standard;

        StandardAttributes(
            const std::string& name = "std-th",
            const CpuAffinitySet& cpuAffinities = CpuAffinity::getDefault(),
            StackSize stackSize = StackSize::getDefault(),
            SchedulingPolicy policy = scheduling_policy::Standard::OTHER,
            Nice nice = Nice::getDefault());

        SchedulingPolicy getPolicy() const
        {
            return m_policy;
        }

        Nice getNice() const
        {
            return m_nice;
        }

        std::string getPolicyName() const;

        bool isValid() const;

    private:
        SchedulingPolicy m_policy;
        Nice m_nice;
    };

}}

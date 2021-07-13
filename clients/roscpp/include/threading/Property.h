/***************************************************************************************************
 * Copyright Five AI 2021.
 * All rights reserved.
 ***************************************************************************************************/
#pragma once

#include <boost/operators.hpp>
#include <boost/type_index.hpp>

#include <stdexcept>
#include <string>

namespace ros { namespace threading
{
    template <typename T, class D>
    class Property : boost::totally_ordered<D,
                      boost::addable<D,
                      boost::subtractable<D>> >
    {
    public:
        using Value = T;

        explicit Property(Value value) :
            m_value{value}
        {}

        Value getValue() const
        {
            return m_value;
        }

        bool setValue(Value value)
        {
            // make a copy of the current value
            const auto old = m_value;

            // change current value
            m_value = value;

            // check if new value is valid and roll it back in case it is not
            if (! derived().isValid())
            {
                m_value = old;
                return false;
            }
            return true;
        }

        void trySetValue(Value value)
        {
            if (! setValue(value))
            {
                const auto msg = "Invalid value " + std::to_string(value) +
                    " for property " + boost::typeindex::type_id<D>().pretty_name();
                throw std::logic_error{msg};
            }
        }

        // NB. other operators which can be implemented in terms of this
        //     one are implicitly inherited from the base class
        bool operator==(const D& rhs) const
        {
            return m_value == rhs.m_value;
        }

        bool operator<(const D& rhs) const
        {
            return m_value < rhs.m_value;
        }

        D& operator+=(const D& rhs)
        {
            trySetValue(getValue() + rhs.getValue());
            return derived();
        }

        D& operator-=(const D& rhs)
        {
            trySetValue(getValue() - rhs.getValue());
            return derived();
        }

    protected:
        // forbid destroying derived classes through pointer to Property
        ~Property() noexcept = default;

    private:
        D& derived()
        {
            return static_cast<D&>(*this);
        }

    private:
        Value m_value;
    };
}}

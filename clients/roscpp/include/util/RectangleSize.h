/***************************************************************************************************
 * Copyright Five AI 2021.
 * All rights reserved.
 ***************************************************************************************************/
#pragma once

namespace fiveai { namespace util
{
    /*
     * Portable and type safe class encapsulating the size of a rectangle.
     * The underlying type is dictated by the template @param T and its
     * units can anything chosen by the client..
    */
    template <typename T>
    class RectangleSize
    {
    public:
        using Type = T;

        constexpr RectangleSize() :
            m_width{0},
            m_height{0}
        {}

        constexpr RectangleSize(Type width, Type height) :
            m_width{width},
            m_height{height}
        {}

        constexpr Type getWidth() const
        {
            return m_width;
        }

        constexpr Type getHeight() const
        {
            return m_height;
        }

        void setWidth(Type val)
        {
            m_width = val;
        }

        void setHeight(Type val)
        {
            m_height = val;
        }

        constexpr Type getArea() const
        {
            return getWidth() * getHeight();
        }

    private:
        Type m_width;
        Type m_height;
    };

    template <typename T>
    bool operator==(const RectangleSize<T>& lhs, const RectangleSize<T>& rhs)
    {
        return (lhs.getWidth()  == rhs.getWidth()) &&
               (lhs.getHeight() == rhs.getHeight());
    }

    template <typename T>
    bool operator!=(const RectangleSize<T>& lhs, const RectangleSize<T>& rhs)
    {
        return !(lhs == rhs);
    }
}}

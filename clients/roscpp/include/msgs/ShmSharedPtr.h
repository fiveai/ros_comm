/***************************************************************************************************
 * Copyright Five AI Limited 2021
 * All rights reserved

 * This file is released under a Creative Commons Attribution-NonCommercial-ShareAlike 4.0
 * International License. Please refer to the README.md file supplied with this repository.
 ***************************************************************************************************/

#pragma once

#include "util/ShmSharedPtr.h"

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <boost/algorithm/string/replace.hpp>

#include <stdexcept>

namespace ros { namespace lot_msgs
{
    template <typename T>
    using SharedPtr = ros::shm::SharedPtr<T>;
}}

namespace ros { namespace message_traits
{
    template <class T> struct IsFixedSize<ros::lot_msgs::SharedPtr<T>> : TrueType {};
    template <class T> struct IsFixedSize<ros::lot_msgs::SharedPtr<T> const> : TrueType {};

    template <class T> struct IsMessage<ros::lot_msgs::SharedPtr<T>> : TrueType {};
    template <class T> struct IsMessage<ros::lot_msgs::SharedPtr<T> const> : TrueType {};

    template <class T> struct HasHeader<ros::lot_msgs::SharedPtr<T>> : FalseType {};
    template <class T> struct HasHeader<ros::lot_msgs::SharedPtr<T> const> : FalseType {};

    template<class T>
    struct MD5Sum<ros::lot_msgs::SharedPtr<T>>
    {
        static const char* value()
        {
            static const auto sum = std::string("1d4399e98b95735c1dfdd7e1c0f7f555") + MD5Sum<T>::value();
            return sum.c_str();
        }

        static const char* value(const ros::lot_msgs::SharedPtr<T>&)
        {
            return value();
        }

        static const uint64_t static_value1 = 0x060021388200f6f0ULL;
        static const uint64_t static_value2 = 0xf447d0fcd9c64743ULL;
    };

    template<class T>
    struct DataType<ros::lot_msgs::SharedPtr<T>>
    {
        static std::string getPointedTypeName()
        {
            const std::string str = DataType<T>::value();
            return boost::replace_all_copy(str, "/", "_");
        }

        static const char* value()
        {
            static const auto v = std::string("sensor_msgs/ShmSharedPtr_") + getPointedTypeName();
            return v.c_str();
        }

        static const char* value(const ros::lot_msgs::SharedPtr<T>&)
        {
            return value();
        }
    };

    template<class T>
    struct Definition<ros::lot_msgs::SharedPtr<T>>
    {
        static const char* value()
        {
            return "# This message contains shared memory shared pointer";
        }

        static const char* value(const ros::lot_msgs::SharedPtr<T>&)
        {
           return value();
        }
    };

}} // namespace ros::message_traits

namespace ros { namespace serialization
{
    template<class _T> struct Serializer<ros::lot_msgs::SharedPtr<_T>>
    {
        template<typename Stream, typename T>
        inline static
        void allInOne(Stream&, T)
        {
            throw std::logic_error{"Shared memory shared pointers are not (de)serializable!"
                                   " This means that they cannot be passed between nodes via "
                                   "ROS publish/subscribe mechanisms."};
        }

        ROS_DECLARE_ALLINONE_SERIALIZER
    };

}} // namespace ros::serialization

namespace ros { namespace message_operations
{
    template<class T>
    struct Printer<ros::lot_msgs::SharedPtr<T>>
    {
        template<typename Stream>
        static
        void stream(Stream& s, const std::string& indent, const ros::lot_msgs::SharedPtr<T>& p)
        {
            s << indent << "pointer value: " << p.get() << std::endl;
            s << indent << "use count: " << p.use_count() << std::endl;
        }
    };

}} // namespace ros::message_operations

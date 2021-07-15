/***************************************************************************************************
 * Copyright Five AI 2021.
 * All rights reserved.
 ***************************************************************************************************/

#pragma once

#include "msgs/ShmHeader.h"
#include "util/ShmString.h"
#include "util/ShmVector.h"
#include "util/ShmSharedPtr.h"

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <boost/interprocess/managed_shared_memory.hpp>

#include <vector>

namespace fiveai { namespace shm_msgs
{
    template <class Sa>
    struct Image_
    {
        using StringAllocator = Sa;
        using DataAllocator   = typename StringAllocator::template rebind<uint8_t>::other;

        using Type            = Image_<StringAllocator>;

        using SegmentManager = boost::interprocess::managed_shared_memory::segment_manager;
        using Header         = shm_msgs::Header_<StringAllocator>;
        using String         = fiveai::shm::_String<typename StringAllocator::value_type, StringAllocator>;
        using Vector         = fiveai::shm::Vector<typename DataAllocator::value_type, DataAllocator>;
        using Size           = uint32_t;
        using Bool           = uint8_t;

        explicit Image_(SegmentManager& segmentManager) :
            header{&segmentManager},
            height{0},
            width{0},
            encoding{StringAllocator{&segmentManager}},
            is_bigendian{0},
            step{0},
            data{DataAllocator{&segmentManager}}
        {}

        Image_(SegmentManager& segmentManager,
               const Header& header,
               const Size height, const Size width,
               const char* encoding, const bool isBigEndian,
               const Size step, const std::vector<uint8_t>& data) :
            header{header},
            height{height},
            width{width},
            encoding{encoding, StringAllocator{&segmentManager}},
            is_bigendian{isBigEndian},
            step{step},
            data{std::begin(data), std::end(data), DataAllocator{&segmentManager}}
        {}

        Image_(const StringAllocator& stringAllocator) :
            header{stringAllocator},
            height{0},
            width{0},
            encoding{stringAllocator},
            is_bigendian{0},
            step{0},
            data{stringAllocator}
        {}

        Header  header;
        Size    height;
        Size    width;
        String  encoding;
        Bool    is_bigendian;
        Size    step;
        Vector  data;
    };

    using Image                  = shm_msgs::Image_<fiveai::shm::StringAllocator>;
    using SharedPtrShmImage      = fiveai::shm::SharedPtr<shm_msgs::Image>;
    using SharedPtrConstShmImage = fiveai::shm::SharedPtr<const shm_msgs::Image>;

    template<typename A>
    std::ostream& operator<<(std::ostream& s, const shm_msgs::Image_<A> & v)
    {
        ros::message_operations::Printer<shm_msgs::Image_<A>>::stream(s, "", v);
        return s;
    }

}}

namespace ros { namespace message_traits
{
    template <class A> struct IsFixedSize<fiveai::shm_msgs::Image_<A>> : FalseType {};
    template <class A> struct IsFixedSize<fiveai::shm_msgs::Image_<A> const> : FalseType {};

    template <class A> struct IsMessage<fiveai::shm_msgs::Image_<A>> : TrueType {};
    template <class A> struct IsMessage<fiveai::shm_msgs::Image_<A> const>: TrueType {};

    template <class A> struct HasHeader<fiveai::shm_msgs::Image_<A>> : TrueType {};
    template <class A> struct HasHeader<fiveai::shm_msgs::Image_<A> const> : TrueType {};

    template<class A>
    struct MD5Sum<fiveai::shm_msgs::Image_<A>>
    {
        static const char* value()
        {
            return "b5370f33e6f7ca0826f9f9522c20aee0";
        }

        static const char* value(const fiveai::shm_msgs::Image_<A>&)
        {
            return value();
        }

        static const uint64_t static_value1 = 0x060021388200f6f0ULL;
        static const uint64_t static_value2 = 0xf447d0fcd9c64743ULL;
    };

    template<class A>
    struct MD5Sum<const fiveai::shm_msgs::Image_<A>>
    {
        static const char* value()
        {
            return "44579588cc1b1465f0f5e0db924d97f8";
        }

        static const char* value(const fiveai::shm_msgs::Image_<A>&)
        {
            return value();
        }

        static const uint64_t static_value1 = 0x060021388200f6f0ULL;
        static const uint64_t static_value2 = 0xf447d0fcd9c64743ULL;
    };

    template<class A>
    struct DataType<fiveai::shm_msgs::Image_<A>>
    {
        static const char* value()
        {
            return "sensor_msgs/ShmImage";
        }

        static const char* value(const fiveai::shm_msgs::Image_<A>&)
        {
            return value();
        }
    };

    template<class A>
    struct DataType<const fiveai::shm_msgs::Image_<A>>
    {
        static const char* value()
        {
            return "sensor_msgs/ConstShmImage";
        }

        static const char* value(const fiveai::shm_msgs::Image_<A>&)
        {
            return value();
        }
    };

    template<class A>
    struct Definition<fiveai::shm_msgs::Image_<A>>
    {
        static const char* value()
        {
            return "# This message has the same semantics and layout as std_msgs::Image."
                   " In addition, it can be placed in shared memory and mutated";
        }

        static const char* value(const fiveai::shm_msgs::Image_<A>&)
        {
            return value();
        }
    };

    template<class A>
    struct Definition<const fiveai::shm_msgs::Image_<A>>
    {
        static const char* value()
        {
            return "# This message has the same semantics and layout as std_msgs::Image."
                   " In addition it can be placed in shared memory. Once constructed it"
                   " cannot be mutated.";
        }

        static const char* value(const fiveai::shm_msgs::Image_<A>&)
        {
            return value();
        }
    };

}} // namespace ros::message_traits

namespace ros { namespace serialization
{
    template<class A>
    struct Serializer<fiveai::shm_msgs::Image_<A>>
    {
        template<typename Stream, typename T>
        inline static
        void allInOne(Stream& stream, T m)
        {
            stream.next(m.header);
            stream.next(m.height);
            stream.next(m.width);
            stream.next(m.encoding);
            stream.next(m.is_bigendian);
            stream.next(m.step);
            stream.next(m.data);
        }

        ROS_DECLARE_ALLINONE_SERIALIZER
    };

}} // namespace ros::serialization

namespace ros { namespace message_operations
{
    template<class A>
    struct Printer<fiveai::shm_msgs::Image_<A>>
    {
        template<typename Stream>
        inline static
        void stream(Stream& s, const std::string& indent, const fiveai::shm_msgs::Image_<A>& v)
        {
            s << indent << "header: ";
            s << std::endl;
            Printer<fiveai::shm_msgs::Header_<A>>::stream(s, indent + "  ", v.header);
            s << indent << "height: ";
            Printer<uint32_t>::stream(s, indent + "  ", v.height);
            s << indent << "width: ";
            Printer<uint32_t>::stream(s, indent + "  ", v.width);
            s << indent << "encoding: ";
            Printer<typename fiveai::shm_msgs::Image_<A>::String>::stream(s, indent + "  ", v.encoding);
            s << indent << "is_bigendian: ";
            Printer<uint8_t>::stream(s, indent + "  ", v.is_bigendian);
            s << indent << "step: ";
            Printer<uint32_t>::stream(s, indent + "  ", v.step);

            s << indent << "data[]" << std::endl;
            for (size_t i = 0; i < v.data.size(); ++i)
            {
                s << indent << "  data[" << i << "]: ";
                Printer<uint8_t>::stream(s, indent + "  ", v.data[i]);
            }
        }
    };

}} // namespace ros::message_operations

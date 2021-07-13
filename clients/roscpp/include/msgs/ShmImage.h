
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

namespace fiveai { namespace std_msgs { namespace shm
{
    template <class Sa>
    struct Image_
    {
        using StringAllocator = Sa;
        using DataAllocator   = typename StringAllocator::template rebind<uint8_t>::other;

        using Type            = Image_<StringAllocator>;

        using SegmentManager = boost::interprocess::managed_shared_memory::segment_manager;
        using Header         = shm::Header_<StringAllocator>;
        using String         = fiveai::platform::shm::_String<typename StringAllocator::value_type, StringAllocator>;
        using Vector         = fiveai::platform::shm::Vector<typename DataAllocator::value_type, DataAllocator>;
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

        // TODO: needs to be clarified
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

    using Image                  = shm::Image_<fiveai::platform::shm::StringAllocator>;
    using SharedPtrShmImage      = fiveai::platform::shm::SharedPtr<shm::Image>;
    using SharedPtrConstShmImage = fiveai::platform::shm::SharedPtr<const shm::Image>;

    // constants requiring out of line definition
    template<typename A>
    std::ostream& operator<<(std::ostream& s, const shm::Image_<A> & v)
    {
        ros::message_operations::Printer<shm::Image_<A>>::stream(s, "", v);
        return s;
    }

}}}

namespace ros { namespace message_traits
{
    template <class A> struct IsFixedSize<fiveai::std_msgs::shm::Image_<A>> : FalseType {};
    template <class A> struct IsFixedSize<fiveai::std_msgs::shm::Image_<A> const> : FalseType {};

    template <class A> struct IsMessage<fiveai::std_msgs::shm::Image_<A>> : TrueType {};
    template <class A> struct IsMessage<fiveai::std_msgs::shm::Image_<A> const>: TrueType {};

    template <class A> struct HasHeader<fiveai::std_msgs::shm::Image_<A>> : TrueType {};
    template <class A> struct HasHeader<fiveai::std_msgs::shm::Image_<A> const> : TrueType {};

    template<class A>
    struct MD5Sum<fiveai::std_msgs::shm::Image_<A>>
    {
        static const char* value()
        {
            // TODO: generate proper MD5 sum
            return "11111111111111111111111111111111";
        }

        static const char* value(const fiveai::std_msgs::shm::Image_<A>&)
        {
            return value();
        }

        static const uint64_t static_value1 = 0x060021388200f6f0ULL;
        static const uint64_t static_value2 = 0xf447d0fcd9c64743ULL;
    };

    template<class A>
    struct MD5Sum<const fiveai::std_msgs::shm::Image_<A>>
    {
        static const char* value()
        {
            // TODO: generate proper MD5 sum
            return "00000000000000000000000000000000";
        }

        static const char* value(const fiveai::std_msgs::shm::Image_<A>&)
        {
            return value();
        }

        static const uint64_t static_value1 = 0x060021388200f6f0ULL;
        static const uint64_t static_value2 = 0xf447d0fcd9c64743ULL;
    };

    template<class A>
    struct DataType<fiveai::std_msgs::shm::Image_<A>>
    {
        static const char* value()
        {
            return "sensor_msgs/ShmImage";
        }

        static const char* value(const fiveai::std_msgs::shm::Image_<A>&)
        {
            return value();
        }
    };

    template<class A>
    struct DataType<const fiveai::std_msgs::shm::Image_<A>>
    {
        static const char* value()
        {
            return "sensor_msgs/ConstShmImage";
        }

        static const char* value(const fiveai::std_msgs::shm::Image_<A>&)
        {
            return value();
        }
    };

    template<class A>
    struct Definition<fiveai::std_msgs::shm::Image_<A>>
    {
        static const char* value()
        {
            return "# This message has the same semantics and layout as std_msgs::Image."
                   " In addition, it can be placed in shared memory and mutated";
        }

        static const char* value(const fiveai::std_msgs::shm::Image_<A>&)
        {
            return value();
        }
    };

    template<class A>
    struct Definition<const fiveai::std_msgs::shm::Image_<A>>
    {
        static const char* value()
        {
            return "# This message has the same semantics and layout as std_msgs::Image."
                   " In addition it can be placed in shared memory. Once constructed it"
                   " cannot be mutated.";
        }

        static const char* value(const fiveai::std_msgs::shm::Image_<A>&)
        {
            return value();
        }
    };

}} // namespace ros::message_traits

namespace ros { namespace serialization
{
    template<class A>
    struct Serializer<fiveai::std_msgs::shm::Image_<A>>
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
    struct Printer<fiveai::std_msgs::shm::Image_<A>>
    {
        template<typename Stream>
        inline static
        void stream(Stream& s, const std::string& indent, const fiveai::std_msgs::shm::Image_<A>& v)
        {
            s << indent << "header: ";
            s << std::endl;
            Printer<fiveai::std_msgs::shm::Header_<A>>::stream(s, indent + "  ", v.header);
            s << indent << "height: ";
            Printer<uint32_t>::stream(s, indent + "  ", v.height);
            s << indent << "width: ";
            Printer<uint32_t>::stream(s, indent + "  ", v.width);
            s << indent << "encoding: ";
            Printer<typename fiveai::std_msgs::shm::Image_<A>::String>::stream(s, indent + "  ", v.encoding);
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

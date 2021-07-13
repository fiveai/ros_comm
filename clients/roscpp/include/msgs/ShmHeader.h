/***************************************************************************************************
 * Copyright Five AI 2020.
 * All rights reserved.
 ***************************************************************************************************/

#pragma once

#include "util/ShmString.h"
#include "util/ShmSharedPtr.h"
#include "util/ShmAllocator.h"

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <boost/interprocess/managed_shared_memory.hpp>

namespace fiveai { namespace std_msgs { namespace shm
{
    template <class Sa>
    struct Header_
    {
        using StringAllocator = Sa;
        using Type            = Header_<StringAllocator>;
        using SegmentManager  = boost::interprocess::managed_shared_memory::segment_manager;
        using Sequence        = uint32_t;
        using Timestamp       = ros::Time;
        using String          = fiveai::platform::shm::_String<typename StringAllocator::value_type, StringAllocator>;

        explicit Header_(SegmentManager& segmentManager) :
            seq{0},
            stamp{},
            frame_id{StringAllocator{&segmentManager}}
        {}

        Header_(SegmentManager& segmentManager, Sequence seq, Timestamp stamp, const char* frameId) :
            seq{seq},
            stamp{stamp},
            frame_id{frameId, StringAllocator{&segmentManager}}
        {}

        Header_(const StringAllocator& stringAllocator) :
            seq{0},
            stamp{},
            frame_id{stringAllocator}
        {}

        Sequence  seq;
        Timestamp stamp;
        String    frame_id;
    };

    using Header            = shm::Header_<fiveai::platform::shm::StringAllocator>;
    using HeaderShmPtr      = fiveai::platform::shm::SharedPtr<shm::Header>;
    using HeaderConstShmPtr = fiveai::platform::shm::SharedPtr<const shm::Header>;

    // constants requiring out of line definition

    template<typename A>
    std::ostream& operator<<(std::ostream& s, const shm::Header_<A>& v)
    {
        ros::message_operations::Printer<shm::Header_<A>>::stream(s, "", v);
        return s;
    }
}}}

namespace ros
{
namespace message_traits
{
// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/tmp/binarydeb/ros-kinetic-std-msgs-0.5.11/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']

template <class ContainerAllocator>
struct IsFixedSize< ::fiveai::std_msgs::shm::Header_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::fiveai::std_msgs::shm::Header_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::fiveai::std_msgs::shm::Header_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::fiveai::std_msgs::shm::Header_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::fiveai::std_msgs::shm::Header_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::fiveai::std_msgs::shm::Header_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::fiveai::std_msgs::shm::Header_<ContainerAllocator> >
{
  static const char* value()
  {
    // TODO: generate proper MD5 sum
    return "12345678912345678912345678912345";
  }

  static const char* value(const ::fiveai::std_msgs::shm::Header_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x2176decaecbce78aULL;
  static const uint64_t static_value2 = 0xbc3b96ef049fabedULL;
};

template<class ContainerAllocator>
struct DataType< ::fiveai::std_msgs::shm::Header_<ContainerAllocator> >
{
  static const char* value()
  {
    return "std_msgs/ShmHeader";
  }

  static const char* value(const ::fiveai::std_msgs::shm::Header_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::fiveai::std_msgs::shm::Header_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Same as std_msgs/Header with addition that it can be placed in shm";
  }

  static const char* value(const ::fiveai::std_msgs::shm::Header_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::fiveai::std_msgs::shm::Header_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.seq);
      stream.next(m.stamp);
      stream.next(m.frame_id);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Header_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::fiveai::std_msgs::shm::Header_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::fiveai::std_msgs::shm::Header_<ContainerAllocator>& v)
  {
    s << indent << "seq: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.seq);
    s << indent << "stamp: ";
    Printer<ros::Time>::stream(s, indent + "  ", v.stamp);
    s << indent << "frame_id: ";
    Printer<typename ::fiveai::std_msgs::shm::Header_<ContainerAllocator>::String>::stream(s, indent + "  ", v.frame_id);
  }
};

} // namespace message_operations
} // namespace ros


/***************************************************************************************************
 * Copyright Five AI 2021.
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

namespace ros { namespace lot_msgs
{
    template <class Sa>
    struct Header_
    {
        using StringAllocator = Sa;
        using Type            = Header_<StringAllocator>;
        using SegmentManager  = boost::interprocess::managed_shared_memory::segment_manager;
        using Sequence        = uint32_t;
        using Timestamp       = ros::Time;
        using String          = ros::shm::_String<typename StringAllocator::value_type, StringAllocator>;

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

    using Header            = lot_msgs::Header_<ros::shm::StringAllocator>;
    using HeaderShmPtr      = ros::shm::SharedPtr<lot_msgs::Header>;
    using HeaderConstShmPtr = ros::shm::SharedPtr<const lot_msgs::Header>;

    template<typename A>
    std::ostream& operator<<(std::ostream& s, const lot_msgs::Header_<A>& v)
    {
        ros::message_operations::Printer<lot_msgs::Header_<A>>::stream(s, "", v);
        return s;
    }
}}

namespace ros
{
namespace message_traits
{

template <class ContainerAllocator>
struct IsFixedSize< lot_msgs::Header_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< lot_msgs::Header_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< lot_msgs::Header_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< lot_msgs::Header_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< lot_msgs::Header_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< lot_msgs::Header_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< lot_msgs::Header_<ContainerAllocator> >
{
  static const char* value()
  {
    return "55bf682044a302f2fdf00c1fcd1a70a9";
  }

  static const char* value(const lot_msgs::Header_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x2176decaecbce78aULL;
  static const uint64_t static_value2 = 0xbc3b96ef049fabedULL;
};

template<class ContainerAllocator>
struct DataType< lot_msgs::Header_<ContainerAllocator> >
{
  static const char* value()
  {
    return "std_msgs/ShmHeader";
  }

  static const char* value(const lot_msgs::Header_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< lot_msgs::Header_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Same as std_msgs/Header with addition that it can be placed in shm";
  }

  static const char* value(const lot_msgs::Header_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< lot_msgs::Header_<ContainerAllocator> >
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
struct Printer< lot_msgs::Header_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const lot_msgs::Header_<ContainerAllocator>& v)
  {
    s << indent << "seq: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.seq);
    s << indent << "stamp: ";
    Printer<ros::Time>::stream(s, indent + "  ", v.stamp);
    s << indent << "frame_id: ";
    Printer<typename lot_msgs::Header_<ContainerAllocator>::String>::stream(s, indent + "  ", v.frame_id);
  }
};

} // namespace message_operations
} // namespace ros


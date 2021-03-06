// Generated by tzc_gencpp from file sensor_msgs/Image.msg
// DO NOT EDIT!


#ifndef TZC_SENSOR_MSGS_MESSAGE_IMAGE_HPP
#define TZC_SENSOR_MSGS_MESSAGE_IMAGE_HPP


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <tzc/tzc_topic.h>
#include <tzc/tzc_object.h>
#include <tzc/tzc_vector.h>

#include <std_msgs/Header.h>

namespace tzc_transport {
namespace sensor_msgs {

struct Image_ {

  typedef ::std_msgs::Header _header_type;
  _header_type header;

  typedef uint32_t _height_type;
  _height_type height;

  typedef uint32_t _width_type;
  _width_type width;

  typedef std::string _encoding_type;
  _encoding_type encoding;

  typedef uint8_t _is_bigendian_type;
  _is_bigendian_type is_bigendian;

  typedef uint32_t _step_type;
  _step_type step;

  typedef vector< uint8_t > _data_type;
  _data_type data;

}; // struct Image_

class Image : public Image_, public BaseMsg {
private:
  // used by publisher, calculate required length of shared memory
  size_t getLength() {
    size_t res = 0;
    res += sizeof(uint8_t) * data.size_;
    return res;
  }
  // used by publisher & subscriber, fill the pointer of tzc_transport::vector with shared memory address
  void fillArray(long handle, ShmMessage * shmmsg) {
    if (shmmsg_)
      shmmsg_->release();

    handle_ = handle;
    shmmsg_ = shmmsg;

    void * tmp = (void *)(shmmsg_ + 1);
    data.ptr_ = (uint8_t *)tmp;
    tmp = (void *)(data.ptr_ + data.size_);
  }

public:
  friend class Publisher< Image >;
  friend class SubscriberCallbackHelper< Image >;

  typedef boost::shared_ptr< Image > Ptr;
  typedef boost::shared_ptr< Image const > ConstPtr;

}; // class Image

typedef boost::shared_ptr< Image > ImagePtr;
typedef boost::shared_ptr< Image const > ImageConstPtr;

} // namespace sensor_msgs
} // namespace tzc_transport

namespace ros {
namespace message_traits {



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'std_msgs': ['/opt/ros/indigo/share/std_msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <>
struct IsFixedSize< ::tzc_transport::sensor_msgs::Image >
  : FalseType {
};

template <>
struct IsFixedSize< ::tzc_transport::sensor_msgs::Image const>
  : FalseType {
};

template <>
struct IsMessage< ::tzc_transport::sensor_msgs::Image >
  : TrueType {
};

template <>
struct IsMessage< ::tzc_transport::sensor_msgs::Image const>
  : TrueType {
};

template <>
struct HasHeader< ::tzc_transport::sensor_msgs::Image >
  : TrueType {
};

template <>
struct HasHeader< ::tzc_transport::sensor_msgs::Image const>
  : TrueType {
};


template <>
struct MD5Sum< ::tzc_transport::sensor_msgs::Image > {
  static const char * value() {
    return "060021388200f6f0f447d0fcd9c64743";
  }

  static const char * value(const ::tzc_transport::sensor_msgs::Image &) {
    return value();
  }
  static const uint64_t static_value1 = 0x060021388200f6f0ULL;
  static const uint64_t static_value2 = 0xf447d0fcd9c64743ULL;
};

template <>
struct DataType< ::tzc_transport::sensor_msgs::Image > {
  static const char * value() {
    return "sensor_msgs/Image";
  }

  static const char * value(const ::tzc_transport::sensor_msgs::Image &) {
    return value();
  }
};

template <>
struct Definition< ::tzc_transport::sensor_msgs::Image > {
  static const char * value() {
    return "# This message contains an uncompressed image\n\
# (0, 0) is at top-left corner of image\n\
#\n\
\n\
Header header        # Header timestamp should be acquisition time of image\n\
                     # Header frame_id should be optical frame of camera\n\
                     # origin of frame should be optical center of cameara\n\
                     # +x should point to the right in the image\n\
                     # +y should point down in the image\n\
                     # +z should point into to plane of the image\n\
                     # If the frame_id here and the frame_id of the CameraInfo\n\
                     # message associated with the image conflict\n\
                     # the behavior is undefined\n\
\n\
uint32 height         # image height, that is, number of rows\n\
uint32 width          # image width, that is, number of columns\n\
\n\
# The legal values for encoding are in file src/image_encodings.cpp\n\
# If you want to standardize a new string format, join\n\
# ros-users@lists.sourceforge.net and send an email proposing a new encoding.\n\
\n\
string encoding       # Encoding of pixels -- channel meaning, ordering, size\n\
                      # taken from the list of strings in include/sensor_msgs/image_encodings.h\n\
\n\
uint8 is_bigendian    # is this data bigendian?\n\
uint32 step           # Full row length in bytes\n\
uint8[] data          # actual matrix data, size is (step * rows)\n\
\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n\
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
";
  }

  static const char * value(const ::tzc_transport::sensor_msgs::Image &) {
    return value();
  }
};

} // namespace message_traits
} // namespace ros

namespace ros {
namespace serialization {

  template <> struct Serializer< ::tzc_transport::sensor_msgs::Image_ > {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m) {
      stream.next(m.header);
      stream.next(m.height);
      stream.next(m.width);
      stream.next(m.encoding);
      stream.next(m.is_bigendian);
      stream.next(m.step);
      stream.next(m.data);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // class Image_

  template <> struct Serializer< ::tzc_transport::sensor_msgs::Image > {
    template <typename Stream, typename T> inline static void allInOne(Stream & stream, T m) {
      stream.next(*(::tzc_transport::sensor_msgs::Image_ *)&m);
      stream.next(*(::tzc_transport::BaseMsg *)&m);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // class Image

} // namespace serialization
} // namespace ros

#endif // TZC_SENSOR_MSGS_MESSAGE_IMAGE_HPP


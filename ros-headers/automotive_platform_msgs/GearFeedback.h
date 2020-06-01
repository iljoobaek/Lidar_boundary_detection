// Generated by gencpp from file automotive_platform_msgs/GearFeedback.msg
// DO NOT EDIT!


#ifndef AUTOMOTIVE_PLATFORM_MSGS_MESSAGE_GEARFEEDBACK_H
#define AUTOMOTIVE_PLATFORM_MSGS_MESSAGE_GEARFEEDBACK_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <automotive_platform_msgs/Gear.h>

namespace automotive_platform_msgs
{
template <class ContainerAllocator>
struct GearFeedback_
{
  typedef GearFeedback_<ContainerAllocator> Type;

  GearFeedback_()
    : header()
    , current_gear()  {
    }
  GearFeedback_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , current_gear(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef  ::automotive_platform_msgs::Gear_<ContainerAllocator>  _current_gear_type;
  _current_gear_type current_gear;





  typedef boost::shared_ptr< ::automotive_platform_msgs::GearFeedback_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::automotive_platform_msgs::GearFeedback_<ContainerAllocator> const> ConstPtr;

}; // struct GearFeedback_

typedef ::automotive_platform_msgs::GearFeedback_<std::allocator<void> > GearFeedback;

typedef boost::shared_ptr< ::automotive_platform_msgs::GearFeedback > GearFeedbackPtr;
typedef boost::shared_ptr< ::automotive_platform_msgs::GearFeedback const> GearFeedbackConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::automotive_platform_msgs::GearFeedback_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::automotive_platform_msgs::GearFeedback_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace automotive_platform_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'automotive_platform_msgs': ['/tmp/binarydeb/ros-kinetic-automotive-platform-msgs-3.0.3/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::automotive_platform_msgs::GearFeedback_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::automotive_platform_msgs::GearFeedback_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::automotive_platform_msgs::GearFeedback_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::automotive_platform_msgs::GearFeedback_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::automotive_platform_msgs::GearFeedback_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::automotive_platform_msgs::GearFeedback_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::automotive_platform_msgs::GearFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "328d639d4b37a5731de132f4aeb55699";
  }

  static const char* value(const ::automotive_platform_msgs::GearFeedback_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x328d639d4b37a573ULL;
  static const uint64_t static_value2 = 0x1de132f4aeb55699ULL;
};

template<class ContainerAllocator>
struct DataType< ::automotive_platform_msgs::GearFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "automotive_platform_msgs/GearFeedback";
  }

  static const char* value(const ::automotive_platform_msgs::GearFeedback_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::automotive_platform_msgs::GearFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# Current Gear Feedback\n\
\n\
std_msgs/Header header\n\
\n\
automotive_platform_msgs/Gear current_gear\n\
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
\n\
================================================================================\n\
MSG: automotive_platform_msgs/Gear\n\
# Transmission Gear Value\n\
# Provides common definitions for commanded and reported gear\n\
\n\
uint8 NONE=0\n\
uint8 PARK=1\n\
uint8 REVERSE=2\n\
uint8 NEUTRAL=3\n\
uint8 DRIVE=4\n\
uint8 LOW=5\n\
uint8 gear\n\
\n\
";
  }

  static const char* value(const ::automotive_platform_msgs::GearFeedback_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::automotive_platform_msgs::GearFeedback_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.current_gear);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct GearFeedback_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::automotive_platform_msgs::GearFeedback_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::automotive_platform_msgs::GearFeedback_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "current_gear: ";
    s << std::endl;
    Printer< ::automotive_platform_msgs::Gear_<ContainerAllocator> >::stream(s, indent + "  ", v.current_gear);
  }
};

} // namespace message_operations
} // namespace ros

#endif // AUTOMOTIVE_PLATFORM_MSGS_MESSAGE_GEARFEEDBACK_H

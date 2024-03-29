// Generated by gencpp from file automotive_platform_msgs/AdaptiveCruiseControlCommand.msg
// DO NOT EDIT!


#ifndef AUTOMOTIVE_PLATFORM_MSGS_MESSAGE_ADAPTIVECRUISECONTROLCOMMAND_H
#define AUTOMOTIVE_PLATFORM_MSGS_MESSAGE_ADAPTIVECRUISECONTROLCOMMAND_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace automotive_platform_msgs
{
template <class ContainerAllocator>
struct AdaptiveCruiseControlCommand_
{
  typedef AdaptiveCruiseControlCommand_<ContainerAllocator> Type;

  AdaptiveCruiseControlCommand_()
    : header()
    , msg_counter(0)
    , set_speed(0.0)
    , set(0)
    , resume(0)
    , cancel(0)
    , speed_up(0)
    , slow_down(0)
    , further(0)
    , closer(0)  {
    }
  AdaptiveCruiseControlCommand_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , msg_counter(0)
    , set_speed(0.0)
    , set(0)
    , resume(0)
    , cancel(0)
    , speed_up(0)
    , slow_down(0)
    , further(0)
    , closer(0)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef uint8_t _msg_counter_type;
  _msg_counter_type msg_counter;

   typedef float _set_speed_type;
  _set_speed_type set_speed;

   typedef uint16_t _set_type;
  _set_type set;

   typedef uint16_t _resume_type;
  _resume_type resume;

   typedef uint16_t _cancel_type;
  _cancel_type cancel;

   typedef uint16_t _speed_up_type;
  _speed_up_type speed_up;

   typedef uint16_t _slow_down_type;
  _slow_down_type slow_down;

   typedef uint16_t _further_type;
  _further_type further;

   typedef uint16_t _closer_type;
  _closer_type closer;





  typedef boost::shared_ptr< ::automotive_platform_msgs::AdaptiveCruiseControlCommand_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::automotive_platform_msgs::AdaptiveCruiseControlCommand_<ContainerAllocator> const> ConstPtr;

}; // struct AdaptiveCruiseControlCommand_

typedef ::automotive_platform_msgs::AdaptiveCruiseControlCommand_<std::allocator<void> > AdaptiveCruiseControlCommand;

typedef boost::shared_ptr< ::automotive_platform_msgs::AdaptiveCruiseControlCommand > AdaptiveCruiseControlCommandPtr;
typedef boost::shared_ptr< ::automotive_platform_msgs::AdaptiveCruiseControlCommand const> AdaptiveCruiseControlCommandConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::automotive_platform_msgs::AdaptiveCruiseControlCommand_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::automotive_platform_msgs::AdaptiveCruiseControlCommand_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::automotive_platform_msgs::AdaptiveCruiseControlCommand_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::automotive_platform_msgs::AdaptiveCruiseControlCommand_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::automotive_platform_msgs::AdaptiveCruiseControlCommand_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::automotive_platform_msgs::AdaptiveCruiseControlCommand_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::automotive_platform_msgs::AdaptiveCruiseControlCommand_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::automotive_platform_msgs::AdaptiveCruiseControlCommand_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::automotive_platform_msgs::AdaptiveCruiseControlCommand_<ContainerAllocator> >
{
  static const char* value()
  {
    return "45cb31c55c795766905c8d3ddf401e18";
  }

  static const char* value(const ::automotive_platform_msgs::AdaptiveCruiseControlCommand_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x45cb31c55c795766ULL;
  static const uint64_t static_value2 = 0x905c8d3ddf401e18ULL;
};

template<class ContainerAllocator>
struct DataType< ::automotive_platform_msgs::AdaptiveCruiseControlCommand_<ContainerAllocator> >
{
  static const char* value()
  {
    return "automotive_platform_msgs/AdaptiveCruiseControlCommand";
  }

  static const char* value(const ::automotive_platform_msgs::AdaptiveCruiseControlCommand_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::automotive_platform_msgs::AdaptiveCruiseControlCommand_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# Adaptive Cruise Control Command Message\n\
# Contains commands to engage/disengage ACC or adjust the set points\n\
\n\
std_msgs/Header header\n\
\n\
uint8 msg_counter   # Increments each time a command is sent\n\
                    # An acknowledge message should be published with this value\n\
\n\
float32 set_speed   # Speed setpoint (m/sec)\n\
uint16 set          # Engage ACC at the above speed set point\n\
uint16 resume       # Resume ACC at previous speed set point\n\
uint16 cancel       # Disengage ACC\n\
uint16 speed_up     # Increase speed set point\n\
uint16 slow_down    # Decrease speed set point\n\
uint16 further      # Increase distance set point\n\
uint16 closer       # Decrease distance set point\n\
\n\
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

  static const char* value(const ::automotive_platform_msgs::AdaptiveCruiseControlCommand_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::automotive_platform_msgs::AdaptiveCruiseControlCommand_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.msg_counter);
      stream.next(m.set_speed);
      stream.next(m.set);
      stream.next(m.resume);
      stream.next(m.cancel);
      stream.next(m.speed_up);
      stream.next(m.slow_down);
      stream.next(m.further);
      stream.next(m.closer);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct AdaptiveCruiseControlCommand_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::automotive_platform_msgs::AdaptiveCruiseControlCommand_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::automotive_platform_msgs::AdaptiveCruiseControlCommand_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "msg_counter: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.msg_counter);
    s << indent << "set_speed: ";
    Printer<float>::stream(s, indent + "  ", v.set_speed);
    s << indent << "set: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.set);
    s << indent << "resume: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.resume);
    s << indent << "cancel: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.cancel);
    s << indent << "speed_up: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.speed_up);
    s << indent << "slow_down: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.slow_down);
    s << indent << "further: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.further);
    s << indent << "closer: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.closer);
  }
};

} // namespace message_operations
} // namespace ros

#endif // AUTOMOTIVE_PLATFORM_MSGS_MESSAGE_ADAPTIVECRUISECONTROLCOMMAND_H

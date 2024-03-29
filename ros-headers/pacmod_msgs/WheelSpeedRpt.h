// Generated by gencpp from file pacmod_msgs/WheelSpeedRpt.msg
// DO NOT EDIT!


#ifndef PACMOD_MSGS_MESSAGE_WHEELSPEEDRPT_H
#define PACMOD_MSGS_MESSAGE_WHEELSPEEDRPT_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace pacmod_msgs
{
template <class ContainerAllocator>
struct WheelSpeedRpt_
{
  typedef WheelSpeedRpt_<ContainerAllocator> Type;

  WheelSpeedRpt_()
    : header()
    , front_left_wheel_speed(0.0)
    , front_right_wheel_speed(0.0)
    , rear_left_wheel_speed(0.0)
    , rear_right_wheel_speed(0.0)  {
    }
  WheelSpeedRpt_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , front_left_wheel_speed(0.0)
    , front_right_wheel_speed(0.0)
    , rear_left_wheel_speed(0.0)
    , rear_right_wheel_speed(0.0)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef double _front_left_wheel_speed_type;
  _front_left_wheel_speed_type front_left_wheel_speed;

   typedef double _front_right_wheel_speed_type;
  _front_right_wheel_speed_type front_right_wheel_speed;

   typedef double _rear_left_wheel_speed_type;
  _rear_left_wheel_speed_type rear_left_wheel_speed;

   typedef double _rear_right_wheel_speed_type;
  _rear_right_wheel_speed_type rear_right_wheel_speed;





  typedef boost::shared_ptr< ::pacmod_msgs::WheelSpeedRpt_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::pacmod_msgs::WheelSpeedRpt_<ContainerAllocator> const> ConstPtr;

}; // struct WheelSpeedRpt_

typedef ::pacmod_msgs::WheelSpeedRpt_<std::allocator<void> > WheelSpeedRpt;

typedef boost::shared_ptr< ::pacmod_msgs::WheelSpeedRpt > WheelSpeedRptPtr;
typedef boost::shared_ptr< ::pacmod_msgs::WheelSpeedRpt const> WheelSpeedRptConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::pacmod_msgs::WheelSpeedRpt_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::pacmod_msgs::WheelSpeedRpt_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace pacmod_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'pacmod_msgs': ['/tmp/binarydeb/ros-kinetic-pacmod-msgs-3.0.1/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::pacmod_msgs::WheelSpeedRpt_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::pacmod_msgs::WheelSpeedRpt_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::pacmod_msgs::WheelSpeedRpt_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::pacmod_msgs::WheelSpeedRpt_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pacmod_msgs::WheelSpeedRpt_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pacmod_msgs::WheelSpeedRpt_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::pacmod_msgs::WheelSpeedRpt_<ContainerAllocator> >
{
  static const char* value()
  {
    return "0ade552ffaaff87eca01c4d9b49bb8ae";
  }

  static const char* value(const ::pacmod_msgs::WheelSpeedRpt_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x0ade552ffaaff87eULL;
  static const uint64_t static_value2 = 0xca01c4d9b49bb8aeULL;
};

template<class ContainerAllocator>
struct DataType< ::pacmod_msgs::WheelSpeedRpt_<ContainerAllocator> >
{
  static const char* value()
  {
    return "pacmod_msgs/WheelSpeedRpt";
  }

  static const char* value(const ::pacmod_msgs::WheelSpeedRpt_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::pacmod_msgs::WheelSpeedRpt_<ContainerAllocator> >
{
  static const char* value()
  {
    return "std_msgs/Header header\n\
\n\
float64 front_left_wheel_speed\n\
float64 front_right_wheel_speed\n\
float64 rear_left_wheel_speed\n\
float64 rear_right_wheel_speed\n\
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

  static const char* value(const ::pacmod_msgs::WheelSpeedRpt_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::pacmod_msgs::WheelSpeedRpt_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.front_left_wheel_speed);
      stream.next(m.front_right_wheel_speed);
      stream.next(m.rear_left_wheel_speed);
      stream.next(m.rear_right_wheel_speed);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct WheelSpeedRpt_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::pacmod_msgs::WheelSpeedRpt_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::pacmod_msgs::WheelSpeedRpt_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "front_left_wheel_speed: ";
    Printer<double>::stream(s, indent + "  ", v.front_left_wheel_speed);
    s << indent << "front_right_wheel_speed: ";
    Printer<double>::stream(s, indent + "  ", v.front_right_wheel_speed);
    s << indent << "rear_left_wheel_speed: ";
    Printer<double>::stream(s, indent + "  ", v.rear_left_wheel_speed);
    s << indent << "rear_right_wheel_speed: ";
    Printer<double>::stream(s, indent + "  ", v.rear_right_wheel_speed);
  }
};

} // namespace message_operations
} // namespace ros

#endif // PACMOD_MSGS_MESSAGE_WHEELSPEEDRPT_H

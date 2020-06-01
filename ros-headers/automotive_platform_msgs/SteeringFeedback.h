// Generated by gencpp from file automotive_platform_msgs/SteeringFeedback.msg
// DO NOT EDIT!


#ifndef AUTOMOTIVE_PLATFORM_MSGS_MESSAGE_STEERINGFEEDBACK_H
#define AUTOMOTIVE_PLATFORM_MSGS_MESSAGE_STEERINGFEEDBACK_H


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
struct SteeringFeedback_
{
  typedef SteeringFeedback_<ContainerAllocator> Type;

  SteeringFeedback_()
    : header()
    , steering_wheel_angle(0.0)  {
    }
  SteeringFeedback_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , steering_wheel_angle(0.0)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef float _steering_wheel_angle_type;
  _steering_wheel_angle_type steering_wheel_angle;





  typedef boost::shared_ptr< ::automotive_platform_msgs::SteeringFeedback_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::automotive_platform_msgs::SteeringFeedback_<ContainerAllocator> const> ConstPtr;

}; // struct SteeringFeedback_

typedef ::automotive_platform_msgs::SteeringFeedback_<std::allocator<void> > SteeringFeedback;

typedef boost::shared_ptr< ::automotive_platform_msgs::SteeringFeedback > SteeringFeedbackPtr;
typedef boost::shared_ptr< ::automotive_platform_msgs::SteeringFeedback const> SteeringFeedbackConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::automotive_platform_msgs::SteeringFeedback_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::automotive_platform_msgs::SteeringFeedback_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::automotive_platform_msgs::SteeringFeedback_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::automotive_platform_msgs::SteeringFeedback_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::automotive_platform_msgs::SteeringFeedback_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::automotive_platform_msgs::SteeringFeedback_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::automotive_platform_msgs::SteeringFeedback_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::automotive_platform_msgs::SteeringFeedback_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::automotive_platform_msgs::SteeringFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "f61fd13efbfee4ea479942d1f6acebc4";
  }

  static const char* value(const ::automotive_platform_msgs::SteeringFeedback_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xf61fd13efbfee4eaULL;
  static const uint64_t static_value2 = 0x479942d1f6acebc4ULL;
};

template<class ContainerAllocator>
struct DataType< ::automotive_platform_msgs::SteeringFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "automotive_platform_msgs/SteeringFeedback";
  }

  static const char* value(const ::automotive_platform_msgs::SteeringFeedback_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::automotive_platform_msgs::SteeringFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# Current Steering Wheel Angle Feedback\n\
\n\
std_msgs/Header header\n\
\n\
float32 steering_wheel_angle     # Radians\n\
\n\
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

  static const char* value(const ::automotive_platform_msgs::SteeringFeedback_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::automotive_platform_msgs::SteeringFeedback_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.steering_wheel_angle);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct SteeringFeedback_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::automotive_platform_msgs::SteeringFeedback_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::automotive_platform_msgs::SteeringFeedback_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "steering_wheel_angle: ";
    Printer<float>::stream(s, indent + "  ", v.steering_wheel_angle);
  }
};

} // namespace message_operations
} // namespace ros

#endif // AUTOMOTIVE_PLATFORM_MSGS_MESSAGE_STEERINGFEEDBACK_H

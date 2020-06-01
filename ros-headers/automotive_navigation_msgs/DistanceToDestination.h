// Generated by gencpp from file automotive_navigation_msgs/DistanceToDestination.msg
// DO NOT EDIT!


#ifndef AUTOMOTIVE_NAVIGATION_MSGS_MESSAGE_DISTANCETODESTINATION_H
#define AUTOMOTIVE_NAVIGATION_MSGS_MESSAGE_DISTANCETODESTINATION_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace automotive_navigation_msgs
{
template <class ContainerAllocator>
struct DistanceToDestination_
{
  typedef DistanceToDestination_<ContainerAllocator> Type;

  DistanceToDestination_()
    : header()
    , msg_counter(0)
    , distance(0.0)  {
    }
  DistanceToDestination_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , msg_counter(0)
    , distance(0.0)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef uint8_t _msg_counter_type;
  _msg_counter_type msg_counter;

   typedef float _distance_type;
  _distance_type distance;





  typedef boost::shared_ptr< ::automotive_navigation_msgs::DistanceToDestination_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::automotive_navigation_msgs::DistanceToDestination_<ContainerAllocator> const> ConstPtr;

}; // struct DistanceToDestination_

typedef ::automotive_navigation_msgs::DistanceToDestination_<std::allocator<void> > DistanceToDestination;

typedef boost::shared_ptr< ::automotive_navigation_msgs::DistanceToDestination > DistanceToDestinationPtr;
typedef boost::shared_ptr< ::automotive_navigation_msgs::DistanceToDestination const> DistanceToDestinationConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::automotive_navigation_msgs::DistanceToDestination_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::automotive_navigation_msgs::DistanceToDestination_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace automotive_navigation_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg'], 'automotive_navigation_msgs': ['/tmp/binarydeb/ros-kinetic-automotive-navigation-msgs-3.0.3/msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::automotive_navigation_msgs::DistanceToDestination_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::automotive_navigation_msgs::DistanceToDestination_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::automotive_navigation_msgs::DistanceToDestination_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::automotive_navigation_msgs::DistanceToDestination_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::automotive_navigation_msgs::DistanceToDestination_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::automotive_navigation_msgs::DistanceToDestination_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::automotive_navigation_msgs::DistanceToDestination_<ContainerAllocator> >
{
  static const char* value()
  {
    return "5900bd7ec809d5e8d6ec47b8ddef8d1b";
  }

  static const char* value(const ::automotive_navigation_msgs::DistanceToDestination_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x5900bd7ec809d5e8ULL;
  static const uint64_t static_value2 = 0xd6ec47b8ddef8d1bULL;
};

template<class ContainerAllocator>
struct DataType< ::automotive_navigation_msgs::DistanceToDestination_<ContainerAllocator> >
{
  static const char* value()
  {
    return "automotive_navigation_msgs/DistanceToDestination";
  }

  static const char* value(const ::automotive_navigation_msgs::DistanceToDestination_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::automotive_navigation_msgs::DistanceToDestination_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# Distance To Destination Message\n\
# Contains the distance to the desired destination\n\
\n\
std_msgs/Header header\n\
\n\
uint8 msg_counter   # Increments each time a command is sent\n\
                    # An acknowledge message should be published with this value\n\
\n\
float32 distance    # Distance to destination (m)\n\
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

  static const char* value(const ::automotive_navigation_msgs::DistanceToDestination_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::automotive_navigation_msgs::DistanceToDestination_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.msg_counter);
      stream.next(m.distance);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct DistanceToDestination_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::automotive_navigation_msgs::DistanceToDestination_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::automotive_navigation_msgs::DistanceToDestination_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "msg_counter: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.msg_counter);
    s << indent << "distance: ";
    Printer<float>::stream(s, indent + "  ", v.distance);
  }
};

} // namespace message_operations
} // namespace ros

#endif // AUTOMOTIVE_NAVIGATION_MSGS_MESSAGE_DISTANCETODESTINATION_H

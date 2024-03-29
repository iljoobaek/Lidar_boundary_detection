// Generated by gencpp from file pacmod_msgs/DetectedObjectRpt.msg
// DO NOT EDIT!


#ifndef PACMOD_MSGS_MESSAGE_DETECTEDOBJECTRPT_H
#define PACMOD_MSGS_MESSAGE_DETECTEDOBJECTRPT_H


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
struct DetectedObjectRpt_
{
  typedef DetectedObjectRpt_<ContainerAllocator> Type;

  DetectedObjectRpt_()
    : header()
    , front_object_distance_low_res(0.0)
    , front_object_distance_high_res(0.0)  {
    }
  DetectedObjectRpt_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , front_object_distance_low_res(0.0)
    , front_object_distance_high_res(0.0)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef double _front_object_distance_low_res_type;
  _front_object_distance_low_res_type front_object_distance_low_res;

   typedef double _front_object_distance_high_res_type;
  _front_object_distance_high_res_type front_object_distance_high_res;





  typedef boost::shared_ptr< ::pacmod_msgs::DetectedObjectRpt_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::pacmod_msgs::DetectedObjectRpt_<ContainerAllocator> const> ConstPtr;

}; // struct DetectedObjectRpt_

typedef ::pacmod_msgs::DetectedObjectRpt_<std::allocator<void> > DetectedObjectRpt;

typedef boost::shared_ptr< ::pacmod_msgs::DetectedObjectRpt > DetectedObjectRptPtr;
typedef boost::shared_ptr< ::pacmod_msgs::DetectedObjectRpt const> DetectedObjectRptConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::pacmod_msgs::DetectedObjectRpt_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::pacmod_msgs::DetectedObjectRpt_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::pacmod_msgs::DetectedObjectRpt_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::pacmod_msgs::DetectedObjectRpt_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::pacmod_msgs::DetectedObjectRpt_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::pacmod_msgs::DetectedObjectRpt_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pacmod_msgs::DetectedObjectRpt_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pacmod_msgs::DetectedObjectRpt_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::pacmod_msgs::DetectedObjectRpt_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bc712b16be2ea6767f3e682c00a854bc";
  }

  static const char* value(const ::pacmod_msgs::DetectedObjectRpt_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xbc712b16be2ea676ULL;
  static const uint64_t static_value2 = 0x7f3e682c00a854bcULL;
};

template<class ContainerAllocator>
struct DataType< ::pacmod_msgs::DetectedObjectRpt_<ContainerAllocator> >
{
  static const char* value()
  {
    return "pacmod_msgs/DetectedObjectRpt";
  }

  static const char* value(const ::pacmod_msgs::DetectedObjectRpt_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::pacmod_msgs::DetectedObjectRpt_<ContainerAllocator> >
{
  static const char* value()
  {
    return "std_msgs/Header header\n\
\n\
float64 front_object_distance_low_res         # Measured distance to nearest front object\n\
float64 front_object_distance_high_res        # Measured distance to nearest front object (high res)\n\
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

  static const char* value(const ::pacmod_msgs::DetectedObjectRpt_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::pacmod_msgs::DetectedObjectRpt_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.front_object_distance_low_res);
      stream.next(m.front_object_distance_high_res);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct DetectedObjectRpt_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::pacmod_msgs::DetectedObjectRpt_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::pacmod_msgs::DetectedObjectRpt_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "front_object_distance_low_res: ";
    Printer<double>::stream(s, indent + "  ", v.front_object_distance_low_res);
    s << indent << "front_object_distance_high_res: ";
    Printer<double>::stream(s, indent + "  ", v.front_object_distance_high_res);
  }
};

} // namespace message_operations
} // namespace ros

#endif // PACMOD_MSGS_MESSAGE_DETECTEDOBJECTRPT_H

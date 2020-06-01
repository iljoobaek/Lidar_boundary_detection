// Generated by gencpp from file jsk_hark_msgs/HarkPower.msg
// DO NOT EDIT!


#ifndef JSK_HARK_MSGS_MESSAGE_HARKPOWER_H
#define JSK_HARK_MSGS_MESSAGE_HARKPOWER_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace jsk_hark_msgs
{
template <class ContainerAllocator>
struct HarkPower_
{
  typedef HarkPower_<ContainerAllocator> Type;

  HarkPower_()
    : header()
    , count(0)
    , directions(0)
    , data_bytes(0)
    , powers()  {
    }
  HarkPower_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , count(0)
    , directions(0)
    , data_bytes(0)
    , powers(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef int32_t _count_type;
  _count_type count;

   typedef int32_t _directions_type;
  _directions_type directions;

   typedef int32_t _data_bytes_type;
  _data_bytes_type data_bytes;

   typedef std::vector<float, typename ContainerAllocator::template rebind<float>::other >  _powers_type;
  _powers_type powers;





  typedef boost::shared_ptr< ::jsk_hark_msgs::HarkPower_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::jsk_hark_msgs::HarkPower_<ContainerAllocator> const> ConstPtr;

}; // struct HarkPower_

typedef ::jsk_hark_msgs::HarkPower_<std::allocator<void> > HarkPower;

typedef boost::shared_ptr< ::jsk_hark_msgs::HarkPower > HarkPowerPtr;
typedef boost::shared_ptr< ::jsk_hark_msgs::HarkPower const> HarkPowerConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::jsk_hark_msgs::HarkPower_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::jsk_hark_msgs::HarkPower_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace jsk_hark_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'jsk_hark_msgs': ['/tmp/binarydeb/ros-kinetic-jsk-hark-msgs-4.3.1/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::jsk_hark_msgs::HarkPower_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::jsk_hark_msgs::HarkPower_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::jsk_hark_msgs::HarkPower_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::jsk_hark_msgs::HarkPower_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::jsk_hark_msgs::HarkPower_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::jsk_hark_msgs::HarkPower_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::jsk_hark_msgs::HarkPower_<ContainerAllocator> >
{
  static const char* value()
  {
    return "251c13d7a8be27144a2b24c6f53df705";
  }

  static const char* value(const ::jsk_hark_msgs::HarkPower_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x251c13d7a8be2714ULL;
  static const uint64_t static_value2 = 0x4a2b24c6f53df705ULL;
};

template<class ContainerAllocator>
struct DataType< ::jsk_hark_msgs::HarkPower_<ContainerAllocator> >
{
  static const char* value()
  {
    return "jsk_hark_msgs/HarkPower";
  }

  static const char* value(const ::jsk_hark_msgs::HarkPower_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::jsk_hark_msgs::HarkPower_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n\
int32  count\n\
int32  directions\n\
int32  data_bytes\n\
float32[] powers\n\
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

  static const char* value(const ::jsk_hark_msgs::HarkPower_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::jsk_hark_msgs::HarkPower_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.count);
      stream.next(m.directions);
      stream.next(m.data_bytes);
      stream.next(m.powers);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct HarkPower_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::jsk_hark_msgs::HarkPower_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::jsk_hark_msgs::HarkPower_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "count: ";
    Printer<int32_t>::stream(s, indent + "  ", v.count);
    s << indent << "directions: ";
    Printer<int32_t>::stream(s, indent + "  ", v.directions);
    s << indent << "data_bytes: ";
    Printer<int32_t>::stream(s, indent + "  ", v.data_bytes);
    s << indent << "powers[]" << std::endl;
    for (size_t i = 0; i < v.powers.size(); ++i)
    {
      s << indent << "  powers[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.powers[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // JSK_HARK_MSGS_MESSAGE_HARKPOWER_H

// Generated by gencpp from file grid_map_msgs/SetGridMapResponse.msg
// DO NOT EDIT!


#ifndef GRID_MAP_MSGS_MESSAGE_SETGRIDMAPRESPONSE_H
#define GRID_MAP_MSGS_MESSAGE_SETGRIDMAPRESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace grid_map_msgs
{
template <class ContainerAllocator>
struct SetGridMapResponse_
{
  typedef SetGridMapResponse_<ContainerAllocator> Type;

  SetGridMapResponse_()
    {
    }
  SetGridMapResponse_(const ContainerAllocator& _alloc)
    {
  (void)_alloc;
    }







  typedef boost::shared_ptr< ::grid_map_msgs::SetGridMapResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::grid_map_msgs::SetGridMapResponse_<ContainerAllocator> const> ConstPtr;

}; // struct SetGridMapResponse_

typedef ::grid_map_msgs::SetGridMapResponse_<std::allocator<void> > SetGridMapResponse;

typedef boost::shared_ptr< ::grid_map_msgs::SetGridMapResponse > SetGridMapResponsePtr;
typedef boost::shared_ptr< ::grid_map_msgs::SetGridMapResponse const> SetGridMapResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::grid_map_msgs::SetGridMapResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::grid_map_msgs::SetGridMapResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace grid_map_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'grid_map_msgs': ['/tmp/binarydeb/ros-kinetic-grid-map-msgs-1.6.2/msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::grid_map_msgs::SetGridMapResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::grid_map_msgs::SetGridMapResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::grid_map_msgs::SetGridMapResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::grid_map_msgs::SetGridMapResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::grid_map_msgs::SetGridMapResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::grid_map_msgs::SetGridMapResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::grid_map_msgs::SetGridMapResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const ::grid_map_msgs::SetGridMapResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd41d8cd98f00b204ULL;
  static const uint64_t static_value2 = 0xe9800998ecf8427eULL;
};

template<class ContainerAllocator>
struct DataType< ::grid_map_msgs::SetGridMapResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "grid_map_msgs/SetGridMapResponse";
  }

  static const char* value(const ::grid_map_msgs::SetGridMapResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::grid_map_msgs::SetGridMapResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n\
";
  }

  static const char* value(const ::grid_map_msgs::SetGridMapResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::grid_map_msgs::SetGridMapResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream&, T)
    {}

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct SetGridMapResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::grid_map_msgs::SetGridMapResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream&, const std::string&, const ::grid_map_msgs::SetGridMapResponse_<ContainerAllocator>&)
  {}
};

} // namespace message_operations
} // namespace ros

#endif // GRID_MAP_MSGS_MESSAGE_SETGRIDMAPRESPONSE_H

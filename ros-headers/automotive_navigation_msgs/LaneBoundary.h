// Generated by gencpp from file automotive_navigation_msgs/LaneBoundary.msg
// DO NOT EDIT!


#ifndef AUTOMOTIVE_NAVIGATION_MSGS_MESSAGE_LANEBOUNDARY_H
#define AUTOMOTIVE_NAVIGATION_MSGS_MESSAGE_LANEBOUNDARY_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <geometry_msgs/Point.h>

namespace automotive_navigation_msgs
{
template <class ContainerAllocator>
struct LaneBoundary_
{
  typedef LaneBoundary_<ContainerAllocator> Type;

  LaneBoundary_()
    : style(0)
    , color(0)
    , line()  {
    }
  LaneBoundary_(const ContainerAllocator& _alloc)
    : style(0)
    , color(0)
    , line(_alloc)  {
  (void)_alloc;
    }



   typedef uint8_t _style_type;
  _style_type style;

   typedef uint8_t _color_type;
  _color_type color;

   typedef std::vector< ::geometry_msgs::Point_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::geometry_msgs::Point_<ContainerAllocator> >::other >  _line_type;
  _line_type line;



  enum {
    UNKNOWN = 0u,
    SOLID = 1u,
    DASHED = 2u,
    SOLID_DASHED = 3u,
    DASHED_SOLID = 4u,
    SOLID_SOLID = 5u,
    WHITE = 1u,
    YELLOW = 2u,
  };


  typedef boost::shared_ptr< ::automotive_navigation_msgs::LaneBoundary_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::automotive_navigation_msgs::LaneBoundary_<ContainerAllocator> const> ConstPtr;

}; // struct LaneBoundary_

typedef ::automotive_navigation_msgs::LaneBoundary_<std::allocator<void> > LaneBoundary;

typedef boost::shared_ptr< ::automotive_navigation_msgs::LaneBoundary > LaneBoundaryPtr;
typedef boost::shared_ptr< ::automotive_navigation_msgs::LaneBoundary const> LaneBoundaryConstPtr;

// constants requiring out of line definition

   

   

   

   

   

   

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::automotive_navigation_msgs::LaneBoundary_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::automotive_navigation_msgs::LaneBoundary_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace automotive_navigation_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg'], 'automotive_navigation_msgs': ['/tmp/binarydeb/ros-kinetic-automotive-navigation-msgs-3.0.3/msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::automotive_navigation_msgs::LaneBoundary_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::automotive_navigation_msgs::LaneBoundary_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::automotive_navigation_msgs::LaneBoundary_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::automotive_navigation_msgs::LaneBoundary_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::automotive_navigation_msgs::LaneBoundary_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::automotive_navigation_msgs::LaneBoundary_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::automotive_navigation_msgs::LaneBoundary_<ContainerAllocator> >
{
  static const char* value()
  {
    return "abba97913ebab3edef0c7c39a4ea132f";
  }

  static const char* value(const ::automotive_navigation_msgs::LaneBoundary_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xabba97913ebab3edULL;
  static const uint64_t static_value2 = 0xef0c7c39a4ea132fULL;
};

template<class ContainerAllocator>
struct DataType< ::automotive_navigation_msgs::LaneBoundary_<ContainerAllocator> >
{
  static const char* value()
  {
    return "automotive_navigation_msgs/LaneBoundary";
  }

  static const char* value(const ::automotive_navigation_msgs::LaneBoundary_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::automotive_navigation_msgs::LaneBoundary_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# Lane Boundary Message\n\
# Contains an array of points and other details\n\
\n\
uint8 UNKNOWN=0\n\
\n\
uint8 SOLID=1\n\
uint8 DASHED=2\n\
uint8 SOLID_DASHED=3\n\
uint8 DASHED_SOLID=4\n\
uint8 SOLID_SOLID=5\n\
uint8 style\n\
\n\
uint8 WHITE=1\n\
uint8 YELLOW=2\n\
uint8 color\n\
\n\
geometry_msgs/Point[] line\n\
\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Point\n\
# This contains the position of a point in free space\n\
float64 x\n\
float64 y\n\
float64 z\n\
";
  }

  static const char* value(const ::automotive_navigation_msgs::LaneBoundary_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::automotive_navigation_msgs::LaneBoundary_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.style);
      stream.next(m.color);
      stream.next(m.line);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct LaneBoundary_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::automotive_navigation_msgs::LaneBoundary_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::automotive_navigation_msgs::LaneBoundary_<ContainerAllocator>& v)
  {
    s << indent << "style: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.style);
    s << indent << "color: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.color);
    s << indent << "line[]" << std::endl;
    for (size_t i = 0; i < v.line.size(); ++i)
    {
      s << indent << "  line[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::geometry_msgs::Point_<ContainerAllocator> >::stream(s, indent + "    ", v.line[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // AUTOMOTIVE_NAVIGATION_MSGS_MESSAGE_LANEBOUNDARY_H

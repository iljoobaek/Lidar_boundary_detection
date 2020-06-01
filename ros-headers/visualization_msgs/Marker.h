// Generated by gencpp from file visualization_msgs/Marker.msg
// DO NOT EDIT!


#ifndef VISUALIZATION_MSGS_MESSAGE_MARKER_H
#define VISUALIZATION_MSGS_MESSAGE_MARKER_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/ColorRGBA.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/ColorRGBA.h>

namespace visualization_msgs
{
template <class ContainerAllocator>
struct Marker_
{
  typedef Marker_<ContainerAllocator> Type;

  Marker_()
    : header()
    , ns()
    , id(0)
    , type(0)
    , action(0)
    , pose()
    , scale()
    , color()
    , lifetime()
    , frame_locked(false)
    , points()
    , colors()
    , text()
    , mesh_resource()
    , mesh_use_embedded_materials(false)  {
    }
  Marker_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , ns(_alloc)
    , id(0)
    , type(0)
    , action(0)
    , pose(_alloc)
    , scale(_alloc)
    , color(_alloc)
    , lifetime()
    , frame_locked(false)
    , points(_alloc)
    , colors(_alloc)
    , text(_alloc)
    , mesh_resource(_alloc)
    , mesh_use_embedded_materials(false)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _ns_type;
  _ns_type ns;

   typedef int32_t _id_type;
  _id_type id;

   typedef int32_t _type_type;
  _type_type type;

   typedef int32_t _action_type;
  _action_type action;

   typedef  ::geometry_msgs::Pose_<ContainerAllocator>  _pose_type;
  _pose_type pose;

   typedef  ::geometry_msgs::Vector3_<ContainerAllocator>  _scale_type;
  _scale_type scale;

   typedef  ::std_msgs::ColorRGBA_<ContainerAllocator>  _color_type;
  _color_type color;

   typedef ros::Duration _lifetime_type;
  _lifetime_type lifetime;

   typedef uint8_t _frame_locked_type;
  _frame_locked_type frame_locked;

   typedef std::vector< ::geometry_msgs::Point_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::geometry_msgs::Point_<ContainerAllocator> >::other >  _points_type;
  _points_type points;

   typedef std::vector< ::std_msgs::ColorRGBA_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::std_msgs::ColorRGBA_<ContainerAllocator> >::other >  _colors_type;
  _colors_type colors;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _text_type;
  _text_type text;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _mesh_resource_type;
  _mesh_resource_type mesh_resource;

   typedef uint8_t _mesh_use_embedded_materials_type;
  _mesh_use_embedded_materials_type mesh_use_embedded_materials;



  enum {
    ARROW = 0u,
    CUBE = 1u,
    SPHERE = 2u,
    CYLINDER = 3u,
    LINE_STRIP = 4u,
    LINE_LIST = 5u,
    CUBE_LIST = 6u,
    SPHERE_LIST = 7u,
    POINTS = 8u,
    TEXT_VIEW_FACING = 9u,
    MESH_RESOURCE = 10u,
    TRIANGLE_LIST = 11u,
    ADD = 0u,
    MODIFY = 0u,
    DELETE = 2u,
    DELETEALL = 3u,
  };


  typedef boost::shared_ptr< ::visualization_msgs::Marker_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::visualization_msgs::Marker_<ContainerAllocator> const> ConstPtr;

}; // struct Marker_

typedef ::visualization_msgs::Marker_<std::allocator<void> > Marker;

typedef boost::shared_ptr< ::visualization_msgs::Marker > MarkerPtr;
typedef boost::shared_ptr< ::visualization_msgs::Marker const> MarkerConstPtr;

// constants requiring out of line definition

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::visualization_msgs::Marker_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::visualization_msgs::Marker_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace visualization_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg'], 'visualization_msgs': ['/tmp/binarydeb/ros-kinetic-visualization-msgs-1.12.7/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::visualization_msgs::Marker_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::visualization_msgs::Marker_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::visualization_msgs::Marker_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::visualization_msgs::Marker_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::visualization_msgs::Marker_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::visualization_msgs::Marker_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::visualization_msgs::Marker_<ContainerAllocator> >
{
  static const char* value()
  {
    return "4048c9de2a16f4ae8e0538085ebf1b97";
  }

  static const char* value(const ::visualization_msgs::Marker_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x4048c9de2a16f4aeULL;
  static const uint64_t static_value2 = 0x8e0538085ebf1b97ULL;
};

template<class ContainerAllocator>
struct DataType< ::visualization_msgs::Marker_<ContainerAllocator> >
{
  static const char* value()
  {
    return "visualization_msgs/Marker";
  }

  static const char* value(const ::visualization_msgs::Marker_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::visualization_msgs::Marker_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# See http://www.ros.org/wiki/rviz/DisplayTypes/Marker and http://www.ros.org/wiki/rviz/Tutorials/Markers%3A%20Basic%20Shapes for more information on using this message with rviz\n\
\n\
uint8 ARROW=0\n\
uint8 CUBE=1\n\
uint8 SPHERE=2\n\
uint8 CYLINDER=3\n\
uint8 LINE_STRIP=4\n\
uint8 LINE_LIST=5\n\
uint8 CUBE_LIST=6\n\
uint8 SPHERE_LIST=7\n\
uint8 POINTS=8\n\
uint8 TEXT_VIEW_FACING=9\n\
uint8 MESH_RESOURCE=10\n\
uint8 TRIANGLE_LIST=11\n\
\n\
uint8 ADD=0\n\
uint8 MODIFY=0\n\
uint8 DELETE=2\n\
uint8 DELETEALL=3\n\
\n\
Header header                        # header for time/frame information\n\
string ns                            # Namespace to place this object in... used in conjunction with id to create a unique name for the object\n\
int32 id 		                         # object ID useful in conjunction with the namespace for manipulating and deleting the object later\n\
int32 type 		                       # Type of object\n\
int32 action 	                       # 0 add/modify an object, 1 (deprecated), 2 deletes an object, 3 deletes all objects\n\
geometry_msgs/Pose pose                 # Pose of the object\n\
geometry_msgs/Vector3 scale             # Scale of the object 1,1,1 means default (usually 1 meter square)\n\
std_msgs/ColorRGBA color             # Color [0.0-1.0]\n\
duration lifetime                    # How long the object should last before being automatically deleted.  0 means forever\n\
bool frame_locked                    # If this marker should be frame-locked, i.e. retransformed into its frame every timestep\n\
\n\
#Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, ...)\n\
geometry_msgs/Point[] points\n\
#Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, ...)\n\
#number of colors must either be 0 or equal to the number of points\n\
#NOTE: alpha is not yet used\n\
std_msgs/ColorRGBA[] colors\n\
\n\
# NOTE: only used for text markers\n\
string text\n\
\n\
# NOTE: only used for MESH_RESOURCE markers\n\
string mesh_resource\n\
bool mesh_use_embedded_materials\n\
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
MSG: geometry_msgs/Pose\n\
# A representation of pose in free space, composed of position and orientation. \n\
Point position\n\
Quaternion orientation\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Point\n\
# This contains the position of a point in free space\n\
float64 x\n\
float64 y\n\
float64 z\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Quaternion\n\
# This represents an orientation in free space in quaternion form.\n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
float64 w\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Vector3\n\
# This represents a vector in free space. \n\
# It is only meant to represent a direction. Therefore, it does not\n\
# make sense to apply a translation to it (e.g., when applying a \n\
# generic rigid transformation to a Vector3, tf2 will only apply the\n\
# rotation). If you want your data to be translatable too, use the\n\
# geometry_msgs/Point message instead.\n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
================================================================================\n\
MSG: std_msgs/ColorRGBA\n\
float32 r\n\
float32 g\n\
float32 b\n\
float32 a\n\
";
  }

  static const char* value(const ::visualization_msgs::Marker_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::visualization_msgs::Marker_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.ns);
      stream.next(m.id);
      stream.next(m.type);
      stream.next(m.action);
      stream.next(m.pose);
      stream.next(m.scale);
      stream.next(m.color);
      stream.next(m.lifetime);
      stream.next(m.frame_locked);
      stream.next(m.points);
      stream.next(m.colors);
      stream.next(m.text);
      stream.next(m.mesh_resource);
      stream.next(m.mesh_use_embedded_materials);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Marker_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::visualization_msgs::Marker_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::visualization_msgs::Marker_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "ns: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.ns);
    s << indent << "id: ";
    Printer<int32_t>::stream(s, indent + "  ", v.id);
    s << indent << "type: ";
    Printer<int32_t>::stream(s, indent + "  ", v.type);
    s << indent << "action: ";
    Printer<int32_t>::stream(s, indent + "  ", v.action);
    s << indent << "pose: ";
    s << std::endl;
    Printer< ::geometry_msgs::Pose_<ContainerAllocator> >::stream(s, indent + "  ", v.pose);
    s << indent << "scale: ";
    s << std::endl;
    Printer< ::geometry_msgs::Vector3_<ContainerAllocator> >::stream(s, indent + "  ", v.scale);
    s << indent << "color: ";
    s << std::endl;
    Printer< ::std_msgs::ColorRGBA_<ContainerAllocator> >::stream(s, indent + "  ", v.color);
    s << indent << "lifetime: ";
    Printer<ros::Duration>::stream(s, indent + "  ", v.lifetime);
    s << indent << "frame_locked: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.frame_locked);
    s << indent << "points[]" << std::endl;
    for (size_t i = 0; i < v.points.size(); ++i)
    {
      s << indent << "  points[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::geometry_msgs::Point_<ContainerAllocator> >::stream(s, indent + "    ", v.points[i]);
    }
    s << indent << "colors[]" << std::endl;
    for (size_t i = 0; i < v.colors.size(); ++i)
    {
      s << indent << "  colors[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::std_msgs::ColorRGBA_<ContainerAllocator> >::stream(s, indent + "    ", v.colors[i]);
    }
    s << indent << "text: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.text);
    s << indent << "mesh_resource: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.mesh_resource);
    s << indent << "mesh_use_embedded_materials: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.mesh_use_embedded_materials);
  }
};

} // namespace message_operations
} // namespace ros

#endif // VISUALIZATION_MSGS_MESSAGE_MARKER_H

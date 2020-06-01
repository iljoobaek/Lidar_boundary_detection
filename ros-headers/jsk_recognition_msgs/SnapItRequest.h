// Generated by gencpp from file jsk_recognition_msgs/SnapItRequest.msg
// DO NOT EDIT!


#ifndef JSK_RECOGNITION_MSGS_MESSAGE_SNAPITREQUEST_H
#define JSK_RECOGNITION_MSGS_MESSAGE_SNAPITREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3Stamped.h>

namespace jsk_recognition_msgs
{
template <class ContainerAllocator>
struct SnapItRequest_
{
  typedef SnapItRequest_<ContainerAllocator> Type;

  SnapItRequest_()
    : header()
    , model_type(0)
    , target_plane()
    , center()
    , direction()
    , radius(0.0)
    , height(0.0)
    , max_distance(0.0)
    , eps_angle(0.0)  {
    }
  SnapItRequest_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , model_type(0)
    , target_plane(_alloc)
    , center(_alloc)
    , direction(_alloc)
    , radius(0.0)
    , height(0.0)
    , max_distance(0.0)
    , eps_angle(0.0)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef uint8_t _model_type_type;
  _model_type_type model_type;

   typedef  ::geometry_msgs::PolygonStamped_<ContainerAllocator>  _target_plane_type;
  _target_plane_type target_plane;

   typedef  ::geometry_msgs::PointStamped_<ContainerAllocator>  _center_type;
  _center_type center;

   typedef  ::geometry_msgs::Vector3Stamped_<ContainerAllocator>  _direction_type;
  _direction_type direction;

   typedef double _radius_type;
  _radius_type radius;

   typedef double _height_type;
  _height_type height;

   typedef double _max_distance_type;
  _max_distance_type max_distance;

   typedef double _eps_angle_type;
  _eps_angle_type eps_angle;



  enum {
    MODEL_PLANE = 0u,
    MODEL_CYLINDER = 1u,
  };


  typedef boost::shared_ptr< ::jsk_recognition_msgs::SnapItRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::jsk_recognition_msgs::SnapItRequest_<ContainerAllocator> const> ConstPtr;

}; // struct SnapItRequest_

typedef ::jsk_recognition_msgs::SnapItRequest_<std::allocator<void> > SnapItRequest;

typedef boost::shared_ptr< ::jsk_recognition_msgs::SnapItRequest > SnapItRequestPtr;
typedef boost::shared_ptr< ::jsk_recognition_msgs::SnapItRequest const> SnapItRequestConstPtr;

// constants requiring out of line definition

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::jsk_recognition_msgs::SnapItRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::jsk_recognition_msgs::SnapItRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace jsk_recognition_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'pcl_msgs': ['/opt/ros/kinetic/share/pcl_msgs/cmake/../msg'], 'sensor_msgs': ['/opt/ros/kinetic/share/sensor_msgs/cmake/../msg'], 'jsk_footstep_msgs': ['/opt/ros/kinetic/share/jsk_footstep_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg'], 'jsk_recognition_msgs': ['/tmp/binarydeb/ros-kinetic-jsk-recognition-msgs-1.2.9/msg'], 'actionlib_msgs': ['/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::jsk_recognition_msgs::SnapItRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::jsk_recognition_msgs::SnapItRequest_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::jsk_recognition_msgs::SnapItRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::jsk_recognition_msgs::SnapItRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::jsk_recognition_msgs::SnapItRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::jsk_recognition_msgs::SnapItRequest_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::jsk_recognition_msgs::SnapItRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "5733f480694296678d81cff0483b399b";
  }

  static const char* value(const ::jsk_recognition_msgs::SnapItRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x5733f48069429667ULL;
  static const uint64_t static_value2 = 0x8d81cff0483b399bULL;
};

template<class ContainerAllocator>
struct DataType< ::jsk_recognition_msgs::SnapItRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "jsk_recognition_msgs/SnapItRequest";
  }

  static const char* value(const ::jsk_recognition_msgs::SnapItRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::jsk_recognition_msgs::SnapItRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n\
uint8 MODEL_PLANE=0\n\
uint8 MODEL_CYLINDER=1\n\
uint8 model_type\n\
\n\
geometry_msgs/PolygonStamped target_plane\n\
\n\
geometry_msgs/PointStamped center\n\
geometry_msgs/Vector3Stamped direction\n\
float64 radius\n\
float64 height\n\
# parameters, 0 means \n\
float64 max_distance\n\
float64 eps_angle\n\
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
MSG: geometry_msgs/PolygonStamped\n\
# This represents a Polygon with reference coordinate frame and timestamp\n\
Header header\n\
Polygon polygon\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Polygon\n\
#A specification of a polygon where the first and last points are assumed to be connected\n\
Point32[] points\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Point32\n\
# This contains the position of a point in free space(with 32 bits of precision).\n\
# It is recommeded to use Point wherever possible instead of Point32.  \n\
# \n\
# This recommendation is to promote interoperability.  \n\
#\n\
# This message is designed to take up less space when sending\n\
# lots of points at once, as in the case of a PointCloud.  \n\
\n\
float32 x\n\
float32 y\n\
float32 z\n\
================================================================================\n\
MSG: geometry_msgs/PointStamped\n\
# This represents a Point with reference coordinate frame and timestamp\n\
Header header\n\
Point point\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Point\n\
# This contains the position of a point in free space\n\
float64 x\n\
float64 y\n\
float64 z\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Vector3Stamped\n\
# This represents a Vector3 with reference coordinate frame and timestamp\n\
Header header\n\
Vector3 vector\n\
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
";
  }

  static const char* value(const ::jsk_recognition_msgs::SnapItRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::jsk_recognition_msgs::SnapItRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.model_type);
      stream.next(m.target_plane);
      stream.next(m.center);
      stream.next(m.direction);
      stream.next(m.radius);
      stream.next(m.height);
      stream.next(m.max_distance);
      stream.next(m.eps_angle);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct SnapItRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::jsk_recognition_msgs::SnapItRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::jsk_recognition_msgs::SnapItRequest_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "model_type: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.model_type);
    s << indent << "target_plane: ";
    s << std::endl;
    Printer< ::geometry_msgs::PolygonStamped_<ContainerAllocator> >::stream(s, indent + "  ", v.target_plane);
    s << indent << "center: ";
    s << std::endl;
    Printer< ::geometry_msgs::PointStamped_<ContainerAllocator> >::stream(s, indent + "  ", v.center);
    s << indent << "direction: ";
    s << std::endl;
    Printer< ::geometry_msgs::Vector3Stamped_<ContainerAllocator> >::stream(s, indent + "  ", v.direction);
    s << indent << "radius: ";
    Printer<double>::stream(s, indent + "  ", v.radius);
    s << indent << "height: ";
    Printer<double>::stream(s, indent + "  ", v.height);
    s << indent << "max_distance: ";
    Printer<double>::stream(s, indent + "  ", v.max_distance);
    s << indent << "eps_angle: ";
    Printer<double>::stream(s, indent + "  ", v.eps_angle);
  }
};

} // namespace message_operations
} // namespace ros

#endif // JSK_RECOGNITION_MSGS_MESSAGE_SNAPITREQUEST_H

// Generated by gencpp from file jsk_recognition_msgs/WhiteBalancePointsRequest.msg
// DO NOT EDIT!


#ifndef JSK_RECOGNITION_MSGS_MESSAGE_WHITEBALANCEPOINTSREQUEST_H
#define JSK_RECOGNITION_MSGS_MESSAGE_WHITEBALANCEPOINTSREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <sensor_msgs/PointCloud2.h>

namespace jsk_recognition_msgs
{
template <class ContainerAllocator>
struct WhiteBalancePointsRequest_
{
  typedef WhiteBalancePointsRequest_<ContainerAllocator> Type;

  WhiteBalancePointsRequest_()
    : reference_color()
    , input()  {
      reference_color.assign(0.0);
  }
  WhiteBalancePointsRequest_(const ContainerAllocator& _alloc)
    : reference_color()
    , input(_alloc)  {
  (void)_alloc;
      reference_color.assign(0.0);
  }



   typedef boost::array<float, 3>  _reference_color_type;
  _reference_color_type reference_color;

   typedef  ::sensor_msgs::PointCloud2_<ContainerAllocator>  _input_type;
  _input_type input;





  typedef boost::shared_ptr< ::jsk_recognition_msgs::WhiteBalancePointsRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::jsk_recognition_msgs::WhiteBalancePointsRequest_<ContainerAllocator> const> ConstPtr;

}; // struct WhiteBalancePointsRequest_

typedef ::jsk_recognition_msgs::WhiteBalancePointsRequest_<std::allocator<void> > WhiteBalancePointsRequest;

typedef boost::shared_ptr< ::jsk_recognition_msgs::WhiteBalancePointsRequest > WhiteBalancePointsRequestPtr;
typedef boost::shared_ptr< ::jsk_recognition_msgs::WhiteBalancePointsRequest const> WhiteBalancePointsRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::jsk_recognition_msgs::WhiteBalancePointsRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::jsk_recognition_msgs::WhiteBalancePointsRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace jsk_recognition_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'pcl_msgs': ['/opt/ros/kinetic/share/pcl_msgs/cmake/../msg'], 'sensor_msgs': ['/opt/ros/kinetic/share/sensor_msgs/cmake/../msg'], 'jsk_footstep_msgs': ['/opt/ros/kinetic/share/jsk_footstep_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg'], 'jsk_recognition_msgs': ['/tmp/binarydeb/ros-kinetic-jsk-recognition-msgs-1.2.9/msg'], 'actionlib_msgs': ['/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::jsk_recognition_msgs::WhiteBalancePointsRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::jsk_recognition_msgs::WhiteBalancePointsRequest_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::jsk_recognition_msgs::WhiteBalancePointsRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::jsk_recognition_msgs::WhiteBalancePointsRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::jsk_recognition_msgs::WhiteBalancePointsRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::jsk_recognition_msgs::WhiteBalancePointsRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::jsk_recognition_msgs::WhiteBalancePointsRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "c114e8c81fa040c23390d86cd0cb8e3a";
  }

  static const char* value(const ::jsk_recognition_msgs::WhiteBalancePointsRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xc114e8c81fa040c2ULL;
  static const uint64_t static_value2 = 0x3390d86cd0cb8e3aULL;
};

template<class ContainerAllocator>
struct DataType< ::jsk_recognition_msgs::WhiteBalancePointsRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "jsk_recognition_msgs/WhiteBalancePointsRequest";
  }

  static const char* value(const ::jsk_recognition_msgs::WhiteBalancePointsRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::jsk_recognition_msgs::WhiteBalancePointsRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float32[3] reference_color\n\
sensor_msgs/PointCloud2 input\n\
\n\
================================================================================\n\
MSG: sensor_msgs/PointCloud2\n\
# This message holds a collection of N-dimensional points, which may\n\
# contain additional information such as normals, intensity, etc. The\n\
# point data is stored as a binary blob, its layout described by the\n\
# contents of the \"fields\" array.\n\
\n\
# The point cloud data may be organized 2d (image-like) or 1d\n\
# (unordered). Point clouds organized as 2d images may be produced by\n\
# camera depth sensors such as stereo or time-of-flight.\n\
\n\
# Time of sensor data acquisition, and the coordinate frame ID (for 3d\n\
# points).\n\
Header header\n\
\n\
# 2D structure of the point cloud. If the cloud is unordered, height is\n\
# 1 and width is the length of the point cloud.\n\
uint32 height\n\
uint32 width\n\
\n\
# Describes the channels and their layout in the binary data blob.\n\
PointField[] fields\n\
\n\
bool    is_bigendian # Is this data bigendian?\n\
uint32  point_step   # Length of a point in bytes\n\
uint32  row_step     # Length of a row in bytes\n\
uint8[] data         # Actual point data, size is (row_step*height)\n\
\n\
bool is_dense        # True if there are no invalid points\n\
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
MSG: sensor_msgs/PointField\n\
# This message holds the description of one point entry in the\n\
# PointCloud2 message format.\n\
uint8 INT8    = 1\n\
uint8 UINT8   = 2\n\
uint8 INT16   = 3\n\
uint8 UINT16  = 4\n\
uint8 INT32   = 5\n\
uint8 UINT32  = 6\n\
uint8 FLOAT32 = 7\n\
uint8 FLOAT64 = 8\n\
\n\
string name      # Name of field\n\
uint32 offset    # Offset from start of point struct\n\
uint8  datatype  # Datatype enumeration, see above\n\
uint32 count     # How many elements in the field\n\
";
  }

  static const char* value(const ::jsk_recognition_msgs::WhiteBalancePointsRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::jsk_recognition_msgs::WhiteBalancePointsRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.reference_color);
      stream.next(m.input);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct WhiteBalancePointsRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::jsk_recognition_msgs::WhiteBalancePointsRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::jsk_recognition_msgs::WhiteBalancePointsRequest_<ContainerAllocator>& v)
  {
    s << indent << "reference_color[]" << std::endl;
    for (size_t i = 0; i < v.reference_color.size(); ++i)
    {
      s << indent << "  reference_color[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.reference_color[i]);
    }
    s << indent << "input: ";
    s << std::endl;
    Printer< ::sensor_msgs::PointCloud2_<ContainerAllocator> >::stream(s, indent + "  ", v.input);
  }
};

} // namespace message_operations
} // namespace ros

#endif // JSK_RECOGNITION_MSGS_MESSAGE_WHITEBALANCEPOINTSREQUEST_H

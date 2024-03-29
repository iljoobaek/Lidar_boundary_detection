// Generated by gencpp from file jsk_recognition_msgs/SetPointCloud2Response.msg
// DO NOT EDIT!


#ifndef JSK_RECOGNITION_MSGS_MESSAGE_SETPOINTCLOUD2RESPONSE_H
#define JSK_RECOGNITION_MSGS_MESSAGE_SETPOINTCLOUD2RESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace jsk_recognition_msgs
{
template <class ContainerAllocator>
struct SetPointCloud2Response_
{
  typedef SetPointCloud2Response_<ContainerAllocator> Type;

  SetPointCloud2Response_()
    : output()  {
    }
  SetPointCloud2Response_(const ContainerAllocator& _alloc)
    : output(_alloc)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _output_type;
  _output_type output;





  typedef boost::shared_ptr< ::jsk_recognition_msgs::SetPointCloud2Response_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::jsk_recognition_msgs::SetPointCloud2Response_<ContainerAllocator> const> ConstPtr;

}; // struct SetPointCloud2Response_

typedef ::jsk_recognition_msgs::SetPointCloud2Response_<std::allocator<void> > SetPointCloud2Response;

typedef boost::shared_ptr< ::jsk_recognition_msgs::SetPointCloud2Response > SetPointCloud2ResponsePtr;
typedef boost::shared_ptr< ::jsk_recognition_msgs::SetPointCloud2Response const> SetPointCloud2ResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::jsk_recognition_msgs::SetPointCloud2Response_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::jsk_recognition_msgs::SetPointCloud2Response_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::jsk_recognition_msgs::SetPointCloud2Response_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::jsk_recognition_msgs::SetPointCloud2Response_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::jsk_recognition_msgs::SetPointCloud2Response_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::jsk_recognition_msgs::SetPointCloud2Response_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::jsk_recognition_msgs::SetPointCloud2Response_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::jsk_recognition_msgs::SetPointCloud2Response_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::jsk_recognition_msgs::SetPointCloud2Response_<ContainerAllocator> >
{
  static const char* value()
  {
    return "0825d95fdfa2c8f4bbb4e9c74bccd3fd";
  }

  static const char* value(const ::jsk_recognition_msgs::SetPointCloud2Response_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x0825d95fdfa2c8f4ULL;
  static const uint64_t static_value2 = 0xbbb4e9c74bccd3fdULL;
};

template<class ContainerAllocator>
struct DataType< ::jsk_recognition_msgs::SetPointCloud2Response_<ContainerAllocator> >
{
  static const char* value()
  {
    return "jsk_recognition_msgs/SetPointCloud2Response";
  }

  static const char* value(const ::jsk_recognition_msgs::SetPointCloud2Response_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::jsk_recognition_msgs::SetPointCloud2Response_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string output\n\
\n\
";
  }

  static const char* value(const ::jsk_recognition_msgs::SetPointCloud2Response_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::jsk_recognition_msgs::SetPointCloud2Response_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.output);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct SetPointCloud2Response_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::jsk_recognition_msgs::SetPointCloud2Response_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::jsk_recognition_msgs::SetPointCloud2Response_<ContainerAllocator>& v)
  {
    s << indent << "output: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.output);
  }
};

} // namespace message_operations
} // namespace ros

#endif // JSK_RECOGNITION_MSGS_MESSAGE_SETPOINTCLOUD2RESPONSE_H

// Generated by gencpp from file jsk_recognition_msgs/TowerPickUpRequest.msg
// DO NOT EDIT!


#ifndef JSK_RECOGNITION_MSGS_MESSAGE_TOWERPICKUPREQUEST_H
#define JSK_RECOGNITION_MSGS_MESSAGE_TOWERPICKUPREQUEST_H


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
struct TowerPickUpRequest_
{
  typedef TowerPickUpRequest_<ContainerAllocator> Type;

  TowerPickUpRequest_()
    : tower_index(0)  {
    }
  TowerPickUpRequest_(const ContainerAllocator& _alloc)
    : tower_index(0)  {
  (void)_alloc;
    }



   typedef int32_t _tower_index_type;
  _tower_index_type tower_index;





  typedef boost::shared_ptr< ::jsk_recognition_msgs::TowerPickUpRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::jsk_recognition_msgs::TowerPickUpRequest_<ContainerAllocator> const> ConstPtr;

}; // struct TowerPickUpRequest_

typedef ::jsk_recognition_msgs::TowerPickUpRequest_<std::allocator<void> > TowerPickUpRequest;

typedef boost::shared_ptr< ::jsk_recognition_msgs::TowerPickUpRequest > TowerPickUpRequestPtr;
typedef boost::shared_ptr< ::jsk_recognition_msgs::TowerPickUpRequest const> TowerPickUpRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::jsk_recognition_msgs::TowerPickUpRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::jsk_recognition_msgs::TowerPickUpRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace jsk_recognition_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'pcl_msgs': ['/opt/ros/kinetic/share/pcl_msgs/cmake/../msg'], 'sensor_msgs': ['/opt/ros/kinetic/share/sensor_msgs/cmake/../msg'], 'jsk_footstep_msgs': ['/opt/ros/kinetic/share/jsk_footstep_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg'], 'jsk_recognition_msgs': ['/tmp/binarydeb/ros-kinetic-jsk-recognition-msgs-1.2.9/msg'], 'actionlib_msgs': ['/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::jsk_recognition_msgs::TowerPickUpRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::jsk_recognition_msgs::TowerPickUpRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::jsk_recognition_msgs::TowerPickUpRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::jsk_recognition_msgs::TowerPickUpRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::jsk_recognition_msgs::TowerPickUpRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::jsk_recognition_msgs::TowerPickUpRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::jsk_recognition_msgs::TowerPickUpRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "e8bd24109f26b6d833bc4570d67d71c9";
  }

  static const char* value(const ::jsk_recognition_msgs::TowerPickUpRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xe8bd24109f26b6d8ULL;
  static const uint64_t static_value2 = 0x33bc4570d67d71c9ULL;
};

template<class ContainerAllocator>
struct DataType< ::jsk_recognition_msgs::TowerPickUpRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "jsk_recognition_msgs/TowerPickUpRequest";
  }

  static const char* value(const ::jsk_recognition_msgs::TowerPickUpRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::jsk_recognition_msgs::TowerPickUpRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int32 tower_index\n\
";
  }

  static const char* value(const ::jsk_recognition_msgs::TowerPickUpRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::jsk_recognition_msgs::TowerPickUpRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.tower_index);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct TowerPickUpRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::jsk_recognition_msgs::TowerPickUpRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::jsk_recognition_msgs::TowerPickUpRequest_<ContainerAllocator>& v)
  {
    s << indent << "tower_index: ";
    Printer<int32_t>::stream(s, indent + "  ", v.tower_index);
  }
};

} // namespace message_operations
} // namespace ros

#endif // JSK_RECOGNITION_MSGS_MESSAGE_TOWERPICKUPREQUEST_H

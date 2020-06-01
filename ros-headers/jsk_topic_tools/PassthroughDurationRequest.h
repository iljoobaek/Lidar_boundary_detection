// Generated by gencpp from file jsk_topic_tools/PassthroughDurationRequest.msg
// DO NOT EDIT!


#ifndef JSK_TOPIC_TOOLS_MESSAGE_PASSTHROUGHDURATIONREQUEST_H
#define JSK_TOPIC_TOOLS_MESSAGE_PASSTHROUGHDURATIONREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace jsk_topic_tools
{
template <class ContainerAllocator>
struct PassthroughDurationRequest_
{
  typedef PassthroughDurationRequest_<ContainerAllocator> Type;

  PassthroughDurationRequest_()
    : duration()  {
    }
  PassthroughDurationRequest_(const ContainerAllocator& _alloc)
    : duration()  {
  (void)_alloc;
    }



   typedef ros::Duration _duration_type;
  _duration_type duration;





  typedef boost::shared_ptr< ::jsk_topic_tools::PassthroughDurationRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::jsk_topic_tools::PassthroughDurationRequest_<ContainerAllocator> const> ConstPtr;

}; // struct PassthroughDurationRequest_

typedef ::jsk_topic_tools::PassthroughDurationRequest_<std::allocator<void> > PassthroughDurationRequest;

typedef boost::shared_ptr< ::jsk_topic_tools::PassthroughDurationRequest > PassthroughDurationRequestPtr;
typedef boost::shared_ptr< ::jsk_topic_tools::PassthroughDurationRequest const> PassthroughDurationRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::jsk_topic_tools::PassthroughDurationRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::jsk_topic_tools::PassthroughDurationRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace jsk_topic_tools

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'jsk_topic_tools': ['/tmp/binarydeb/ros-kinetic-jsk-topic-tools-2.2.10/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::jsk_topic_tools::PassthroughDurationRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::jsk_topic_tools::PassthroughDurationRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::jsk_topic_tools::PassthroughDurationRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::jsk_topic_tools::PassthroughDurationRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::jsk_topic_tools::PassthroughDurationRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::jsk_topic_tools::PassthroughDurationRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::jsk_topic_tools::PassthroughDurationRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "2aa5b5d494c682527e9e9161e1fa58ac";
  }

  static const char* value(const ::jsk_topic_tools::PassthroughDurationRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x2aa5b5d494c68252ULL;
  static const uint64_t static_value2 = 0x7e9e9161e1fa58acULL;
};

template<class ContainerAllocator>
struct DataType< ::jsk_topic_tools::PassthroughDurationRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "jsk_topic_tools/PassthroughDurationRequest";
  }

  static const char* value(const ::jsk_topic_tools::PassthroughDurationRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::jsk_topic_tools::PassthroughDurationRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "duration duration\n\
";
  }

  static const char* value(const ::jsk_topic_tools::PassthroughDurationRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::jsk_topic_tools::PassthroughDurationRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.duration);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct PassthroughDurationRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::jsk_topic_tools::PassthroughDurationRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::jsk_topic_tools::PassthroughDurationRequest_<ContainerAllocator>& v)
  {
    s << indent << "duration: ";
    Printer<ros::Duration>::stream(s, indent + "  ", v.duration);
  }
};

} // namespace message_operations
} // namespace ros

#endif // JSK_TOPIC_TOOLS_MESSAGE_PASSTHROUGHDURATIONREQUEST_H

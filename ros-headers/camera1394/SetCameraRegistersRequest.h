// Generated by gencpp from file camera1394/SetCameraRegistersRequest.msg
// DO NOT EDIT!


#ifndef CAMERA1394_MESSAGE_SETCAMERAREGISTERSREQUEST_H
#define CAMERA1394_MESSAGE_SETCAMERAREGISTERSREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace camera1394
{
template <class ContainerAllocator>
struct SetCameraRegistersRequest_
{
  typedef SetCameraRegistersRequest_<ContainerAllocator> Type;

  SetCameraRegistersRequest_()
    : type(0)
    , offset(0)
    , value()
    , mode(0)  {
    }
  SetCameraRegistersRequest_(const ContainerAllocator& _alloc)
    : type(0)
    , offset(0)
    , value(_alloc)
    , mode(0)  {
  (void)_alloc;
    }



   typedef uint8_t _type_type;
  _type_type type;

   typedef uint64_t _offset_type;
  _offset_type offset;

   typedef std::vector<uint32_t, typename ContainerAllocator::template rebind<uint32_t>::other >  _value_type;
  _value_type value;

   typedef uint32_t _mode_type;
  _mode_type mode;



  enum {
    TYPE_CONTROL = 0u,
    TYPE_ABSOLUTE = 1u,
    TYPE_FORMAT7 = 2u,
    TYPE_ADVANCED_CONTROL = 3u,
    TYPE_PIO = 4u,
    TYPE_SIO = 5u,
    TYPE_STROBE = 6u,
  };


  typedef boost::shared_ptr< ::camera1394::SetCameraRegistersRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::camera1394::SetCameraRegistersRequest_<ContainerAllocator> const> ConstPtr;

}; // struct SetCameraRegistersRequest_

typedef ::camera1394::SetCameraRegistersRequest_<std::allocator<void> > SetCameraRegistersRequest;

typedef boost::shared_ptr< ::camera1394::SetCameraRegistersRequest > SetCameraRegistersRequestPtr;
typedef boost::shared_ptr< ::camera1394::SetCameraRegistersRequest const> SetCameraRegistersRequestConstPtr;

// constants requiring out of line definition

   

   

   

   

   

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::camera1394::SetCameraRegistersRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::camera1394::SetCameraRegistersRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace camera1394

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::camera1394::SetCameraRegistersRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::camera1394::SetCameraRegistersRequest_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::camera1394::SetCameraRegistersRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::camera1394::SetCameraRegistersRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::camera1394::SetCameraRegistersRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::camera1394::SetCameraRegistersRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::camera1394::SetCameraRegistersRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "703211788a5d6f08f820dfb1792ab51c";
  }

  static const char* value(const ::camera1394::SetCameraRegistersRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x703211788a5d6f08ULL;
  static const uint64_t static_value2 = 0xf820dfb1792ab51cULL;
};

template<class ContainerAllocator>
struct DataType< ::camera1394::SetCameraRegistersRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "camera1394/SetCameraRegistersRequest";
  }

  static const char* value(const ::camera1394::SetCameraRegistersRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::camera1394::SetCameraRegistersRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n\
uint8 TYPE_CONTROL=0\n\
uint8 TYPE_ABSOLUTE=1\n\
uint8 TYPE_FORMAT7=2\n\
uint8 TYPE_ADVANCED_CONTROL=3\n\
uint8 TYPE_PIO=4\n\
uint8 TYPE_SIO=5\n\
uint8 TYPE_STROBE=6\n\
\n\
uint8 type\n\
\n\
uint64 offset\n\
\n\
uint32[] value\n\
\n\
uint32 mode\n\
";
  }

  static const char* value(const ::camera1394::SetCameraRegistersRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::camera1394::SetCameraRegistersRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.type);
      stream.next(m.offset);
      stream.next(m.value);
      stream.next(m.mode);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct SetCameraRegistersRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::camera1394::SetCameraRegistersRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::camera1394::SetCameraRegistersRequest_<ContainerAllocator>& v)
  {
    s << indent << "type: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.type);
    s << indent << "offset: ";
    Printer<uint64_t>::stream(s, indent + "  ", v.offset);
    s << indent << "value[]" << std::endl;
    for (size_t i = 0; i < v.value.size(); ++i)
    {
      s << indent << "  value[" << i << "]: ";
      Printer<uint32_t>::stream(s, indent + "  ", v.value[i]);
    }
    s << indent << "mode: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.mode);
  }
};

} // namespace message_operations
} // namespace ros

#endif // CAMERA1394_MESSAGE_SETCAMERAREGISTERSREQUEST_H

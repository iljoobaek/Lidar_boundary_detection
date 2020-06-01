// Generated by gencpp from file rosapi/ServiceResponseDetailsResponse.msg
// DO NOT EDIT!


#ifndef ROSAPI_MESSAGE_SERVICERESPONSEDETAILSRESPONSE_H
#define ROSAPI_MESSAGE_SERVICERESPONSEDETAILSRESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <rosapi/TypeDef.h>

namespace rosapi
{
template <class ContainerAllocator>
struct ServiceResponseDetailsResponse_
{
  typedef ServiceResponseDetailsResponse_<ContainerAllocator> Type;

  ServiceResponseDetailsResponse_()
    : typedefs()  {
    }
  ServiceResponseDetailsResponse_(const ContainerAllocator& _alloc)
    : typedefs(_alloc)  {
  (void)_alloc;
    }



   typedef std::vector< ::rosapi::TypeDef_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::rosapi::TypeDef_<ContainerAllocator> >::other >  _typedefs_type;
  _typedefs_type typedefs;





  typedef boost::shared_ptr< ::rosapi::ServiceResponseDetailsResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::rosapi::ServiceResponseDetailsResponse_<ContainerAllocator> const> ConstPtr;

}; // struct ServiceResponseDetailsResponse_

typedef ::rosapi::ServiceResponseDetailsResponse_<std::allocator<void> > ServiceResponseDetailsResponse;

typedef boost::shared_ptr< ::rosapi::ServiceResponseDetailsResponse > ServiceResponseDetailsResponsePtr;
typedef boost::shared_ptr< ::rosapi::ServiceResponseDetailsResponse const> ServiceResponseDetailsResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::rosapi::ServiceResponseDetailsResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::rosapi::ServiceResponseDetailsResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace rosapi

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'rosapi': ['/tmp/binarydeb/ros-kinetic-rosapi-0.11.4/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::rosapi::ServiceResponseDetailsResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::rosapi::ServiceResponseDetailsResponse_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::rosapi::ServiceResponseDetailsResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::rosapi::ServiceResponseDetailsResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::rosapi::ServiceResponseDetailsResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::rosapi::ServiceResponseDetailsResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::rosapi::ServiceResponseDetailsResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "a6b8995777f214f2ed97a1e4890feb10";
  }

  static const char* value(const ::rosapi::ServiceResponseDetailsResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xa6b8995777f214f2ULL;
  static const uint64_t static_value2 = 0xed97a1e4890feb10ULL;
};

template<class ContainerAllocator>
struct DataType< ::rosapi::ServiceResponseDetailsResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "rosapi/ServiceResponseDetailsResponse";
  }

  static const char* value(const ::rosapi::ServiceResponseDetailsResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::rosapi::ServiceResponseDetailsResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "TypeDef[] typedefs\n\
\n\
================================================================================\n\
MSG: rosapi/TypeDef\n\
string type\n\
string[] fieldnames\n\
string[] fieldtypes\n\
int32[] fieldarraylen\n\
string[] examples\n\
string[] constnames\n\
string[] constvalues\n\
";
  }

  static const char* value(const ::rosapi::ServiceResponseDetailsResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::rosapi::ServiceResponseDetailsResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.typedefs);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ServiceResponseDetailsResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::rosapi::ServiceResponseDetailsResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::rosapi::ServiceResponseDetailsResponse_<ContainerAllocator>& v)
  {
    s << indent << "typedefs[]" << std::endl;
    for (size_t i = 0; i < v.typedefs.size(); ++i)
    {
      s << indent << "  typedefs[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::rosapi::TypeDef_<ContainerAllocator> >::stream(s, indent + "    ", v.typedefs[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // ROSAPI_MESSAGE_SERVICERESPONSEDETAILSRESPONSE_H
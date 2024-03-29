// Generated by gencpp from file rosapi/TopicsAndRawTypesResponse.msg
// DO NOT EDIT!


#ifndef ROSAPI_MESSAGE_TOPICSANDRAWTYPESRESPONSE_H
#define ROSAPI_MESSAGE_TOPICSANDRAWTYPESRESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace rosapi
{
template <class ContainerAllocator>
struct TopicsAndRawTypesResponse_
{
  typedef TopicsAndRawTypesResponse_<ContainerAllocator> Type;

  TopicsAndRawTypesResponse_()
    : topics()
    , types()
    , typedefs_full_text()  {
    }
  TopicsAndRawTypesResponse_(const ContainerAllocator& _alloc)
    : topics(_alloc)
    , types(_alloc)
    , typedefs_full_text(_alloc)  {
  (void)_alloc;
    }



   typedef std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > , typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::other >  _topics_type;
  _topics_type topics;

   typedef std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > , typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::other >  _types_type;
  _types_type types;

   typedef std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > , typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::other >  _typedefs_full_text_type;
  _typedefs_full_text_type typedefs_full_text;





  typedef boost::shared_ptr< ::rosapi::TopicsAndRawTypesResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::rosapi::TopicsAndRawTypesResponse_<ContainerAllocator> const> ConstPtr;

}; // struct TopicsAndRawTypesResponse_

typedef ::rosapi::TopicsAndRawTypesResponse_<std::allocator<void> > TopicsAndRawTypesResponse;

typedef boost::shared_ptr< ::rosapi::TopicsAndRawTypesResponse > TopicsAndRawTypesResponsePtr;
typedef boost::shared_ptr< ::rosapi::TopicsAndRawTypesResponse const> TopicsAndRawTypesResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::rosapi::TopicsAndRawTypesResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::rosapi::TopicsAndRawTypesResponse_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::rosapi::TopicsAndRawTypesResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::rosapi::TopicsAndRawTypesResponse_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::rosapi::TopicsAndRawTypesResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::rosapi::TopicsAndRawTypesResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::rosapi::TopicsAndRawTypesResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::rosapi::TopicsAndRawTypesResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::rosapi::TopicsAndRawTypesResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "e1432466c8f64316723276ba07c59d12";
  }

  static const char* value(const ::rosapi::TopicsAndRawTypesResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xe1432466c8f64316ULL;
  static const uint64_t static_value2 = 0x723276ba07c59d12ULL;
};

template<class ContainerAllocator>
struct DataType< ::rosapi::TopicsAndRawTypesResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "rosapi/TopicsAndRawTypesResponse";
  }

  static const char* value(const ::rosapi::TopicsAndRawTypesResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::rosapi::TopicsAndRawTypesResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string[] topics\n\
string[] types\n\
string[] typedefs_full_text\n\
\n\
";
  }

  static const char* value(const ::rosapi::TopicsAndRawTypesResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::rosapi::TopicsAndRawTypesResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.topics);
      stream.next(m.types);
      stream.next(m.typedefs_full_text);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct TopicsAndRawTypesResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::rosapi::TopicsAndRawTypesResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::rosapi::TopicsAndRawTypesResponse_<ContainerAllocator>& v)
  {
    s << indent << "topics[]" << std::endl;
    for (size_t i = 0; i < v.topics.size(); ++i)
    {
      s << indent << "  topics[" << i << "]: ";
      Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.topics[i]);
    }
    s << indent << "types[]" << std::endl;
    for (size_t i = 0; i < v.types.size(); ++i)
    {
      s << indent << "  types[" << i << "]: ";
      Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.types[i]);
    }
    s << indent << "typedefs_full_text[]" << std::endl;
    for (size_t i = 0; i < v.typedefs_full_text.size(); ++i)
    {
      s << indent << "  typedefs_full_text[" << i << "]: ";
      Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.typedefs_full_text[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // ROSAPI_MESSAGE_TOPICSANDRAWTYPESRESPONSE_H

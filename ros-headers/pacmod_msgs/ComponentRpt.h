// Generated by gencpp from file pacmod_msgs/ComponentRpt.msg
// DO NOT EDIT!


#ifndef PACMOD_MSGS_MESSAGE_COMPONENTRPT_H
#define PACMOD_MSGS_MESSAGE_COMPONENTRPT_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace pacmod_msgs
{
template <class ContainerAllocator>
struct ComponentRpt_
{
  typedef ComponentRpt_<ContainerAllocator> Type;

  ComponentRpt_()
    : header()
    , component_type(0)
    , component_func(0)
    , counter(0)
    , complement(0)
    , config_fault(false)  {
    }
  ComponentRpt_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , component_type(0)
    , component_func(0)
    , counter(0)
    , complement(0)
    , config_fault(false)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef uint8_t _component_type_type;
  _component_type_type component_type;

   typedef uint8_t _component_func_type;
  _component_func_type component_func;

   typedef uint8_t _counter_type;
  _counter_type counter;

   typedef uint8_t _complement_type;
  _complement_type complement;

   typedef uint8_t _config_fault_type;
  _config_fault_type config_fault;



  enum {
    COMPONENT_TYPE_PACMOD = 0u,
    COMPONENT_TYPE_PACMINI = 1u,
    COMPONENT_TYPE_PACMICRO = 2u,
    COMPONENT_FUNC_PACMOD = 0u,
    COMPONENT_FUNC_STEERING_AND_STEERING_COLUMN = 1u,
    COMPONENT_FUNC_ACCELERATOR_AND_BRAKING = 2u,
    COMPONENT_FUNC_BRAKING = 3u,
    COMPONENT_FUNC_SHIFTING = 4u,
    COMPONENT_FUNC_STEERING = 5u,
    COMPONENT_FUNC_E_SHIFTER = 6u,
    COMPONENT_FUNC_WATCHDOG = 7u,
  };


  typedef boost::shared_ptr< ::pacmod_msgs::ComponentRpt_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::pacmod_msgs::ComponentRpt_<ContainerAllocator> const> ConstPtr;

}; // struct ComponentRpt_

typedef ::pacmod_msgs::ComponentRpt_<std::allocator<void> > ComponentRpt;

typedef boost::shared_ptr< ::pacmod_msgs::ComponentRpt > ComponentRptPtr;
typedef boost::shared_ptr< ::pacmod_msgs::ComponentRpt const> ComponentRptConstPtr;

// constants requiring out of line definition

   

   

   

   

   

   

   

   

   

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::pacmod_msgs::ComponentRpt_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::pacmod_msgs::ComponentRpt_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace pacmod_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'pacmod_msgs': ['/tmp/binarydeb/ros-kinetic-pacmod-msgs-3.0.1/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::pacmod_msgs::ComponentRpt_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::pacmod_msgs::ComponentRpt_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::pacmod_msgs::ComponentRpt_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::pacmod_msgs::ComponentRpt_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pacmod_msgs::ComponentRpt_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pacmod_msgs::ComponentRpt_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::pacmod_msgs::ComponentRpt_<ContainerAllocator> >
{
  static const char* value()
  {
    return "1d5b5cb01195437dc5e2f534206c1017";
  }

  static const char* value(const ::pacmod_msgs::ComponentRpt_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x1d5b5cb01195437dULL;
  static const uint64_t static_value2 = 0xc5e2f534206c1017ULL;
};

template<class ContainerAllocator>
struct DataType< ::pacmod_msgs::ComponentRpt_<ContainerAllocator> >
{
  static const char* value()
  {
    return "pacmod_msgs/ComponentRpt";
  }

  static const char* value(const ::pacmod_msgs::ComponentRpt_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::pacmod_msgs::ComponentRpt_<ContainerAllocator> >
{
  static const char* value()
  {
    return "std_msgs/Header header\n\
\n\
uint8 component_type\n\
uint8 component_func\n\
uint8 counter\n\
uint8 complement\n\
\n\
bool config_fault\n\
\n\
uint8 COMPONENT_TYPE_PACMOD = 0\n\
uint8 COMPONENT_TYPE_PACMINI = 1\n\
uint8 COMPONENT_TYPE_PACMICRO = 2\n\
\n\
uint8 COMPONENT_FUNC_PACMOD = 0\n\
uint8 COMPONENT_FUNC_STEERING_AND_STEERING_COLUMN = 1\n\
uint8 COMPONENT_FUNC_ACCELERATOR_AND_BRAKING = 2\n\
uint8 COMPONENT_FUNC_BRAKING = 3\n\
uint8 COMPONENT_FUNC_SHIFTING = 4\n\
uint8 COMPONENT_FUNC_STEERING = 5\n\
uint8 COMPONENT_FUNC_E_SHIFTER = 6\n\
uint8 COMPONENT_FUNC_WATCHDOG = 7\n\
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
";
  }

  static const char* value(const ::pacmod_msgs::ComponentRpt_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::pacmod_msgs::ComponentRpt_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.component_type);
      stream.next(m.component_func);
      stream.next(m.counter);
      stream.next(m.complement);
      stream.next(m.config_fault);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ComponentRpt_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::pacmod_msgs::ComponentRpt_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::pacmod_msgs::ComponentRpt_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "component_type: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.component_type);
    s << indent << "component_func: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.component_func);
    s << indent << "counter: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.counter);
    s << indent << "complement: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.complement);
    s << indent << "config_fault: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.config_fault);
  }
};

} // namespace message_operations
} // namespace ros

#endif // PACMOD_MSGS_MESSAGE_COMPONENTRPT_H

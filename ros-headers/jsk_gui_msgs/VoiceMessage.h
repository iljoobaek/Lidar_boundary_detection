// Generated by gencpp from file jsk_gui_msgs/VoiceMessage.msg
// DO NOT EDIT!


#ifndef JSK_GUI_MSGS_MESSAGE_VOICEMESSAGE_H
#define JSK_GUI_MSGS_MESSAGE_VOICEMESSAGE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace jsk_gui_msgs
{
template <class ContainerAllocator>
struct VoiceMessage_
{
  typedef VoiceMessage_<ContainerAllocator> Type;

  VoiceMessage_()
    : texts()  {
    }
  VoiceMessage_(const ContainerAllocator& _alloc)
    : texts(_alloc)  {
  (void)_alloc;
    }



   typedef std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > , typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::other >  _texts_type;
  _texts_type texts;





  typedef boost::shared_ptr< ::jsk_gui_msgs::VoiceMessage_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::jsk_gui_msgs::VoiceMessage_<ContainerAllocator> const> ConstPtr;

}; // struct VoiceMessage_

typedef ::jsk_gui_msgs::VoiceMessage_<std::allocator<void> > VoiceMessage;

typedef boost::shared_ptr< ::jsk_gui_msgs::VoiceMessage > VoiceMessagePtr;
typedef boost::shared_ptr< ::jsk_gui_msgs::VoiceMessage const> VoiceMessageConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::jsk_gui_msgs::VoiceMessage_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::jsk_gui_msgs::VoiceMessage_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace jsk_gui_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'sensor_msgs': ['/opt/ros/kinetic/share/sensor_msgs/cmake/../msg'], 'jsk_gui_msgs': ['/tmp/binarydeb/ros-kinetic-jsk-gui-msgs-4.3.1/msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::jsk_gui_msgs::VoiceMessage_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::jsk_gui_msgs::VoiceMessage_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::jsk_gui_msgs::VoiceMessage_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::jsk_gui_msgs::VoiceMessage_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::jsk_gui_msgs::VoiceMessage_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::jsk_gui_msgs::VoiceMessage_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::jsk_gui_msgs::VoiceMessage_<ContainerAllocator> >
{
  static const char* value()
  {
    return "8d7dcfb3b46640ccf02177a3f0cf9530";
  }

  static const char* value(const ::jsk_gui_msgs::VoiceMessage_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x8d7dcfb3b46640ccULL;
  static const uint64_t static_value2 = 0xf02177a3f0cf9530ULL;
};

template<class ContainerAllocator>
struct DataType< ::jsk_gui_msgs::VoiceMessage_<ContainerAllocator> >
{
  static const char* value()
  {
    return "jsk_gui_msgs/VoiceMessage";
  }

  static const char* value(const ::jsk_gui_msgs::VoiceMessage_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::jsk_gui_msgs::VoiceMessage_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string[] texts\n\
";
  }

  static const char* value(const ::jsk_gui_msgs::VoiceMessage_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::jsk_gui_msgs::VoiceMessage_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.texts);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct VoiceMessage_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::jsk_gui_msgs::VoiceMessage_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::jsk_gui_msgs::VoiceMessage_<ContainerAllocator>& v)
  {
    s << indent << "texts[]" << std::endl;
    for (size_t i = 0; i < v.texts.size(); ++i)
    {
      s << indent << "  texts[" << i << "]: ";
      Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.texts[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // JSK_GUI_MSGS_MESSAGE_VOICEMESSAGE_H

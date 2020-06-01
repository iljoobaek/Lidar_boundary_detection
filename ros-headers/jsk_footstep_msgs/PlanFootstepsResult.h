// Generated by gencpp from file jsk_footstep_msgs/PlanFootstepsResult.msg
// DO NOT EDIT!


#ifndef JSK_FOOTSTEP_MSGS_MESSAGE_PLANFOOTSTEPSRESULT_H
#define JSK_FOOTSTEP_MSGS_MESSAGE_PLANFOOTSTEPSRESULT_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <jsk_footstep_msgs/FootstepArray.h>

namespace jsk_footstep_msgs
{
template <class ContainerAllocator>
struct PlanFootstepsResult_
{
  typedef PlanFootstepsResult_<ContainerAllocator> Type;

  PlanFootstepsResult_()
    : result()  {
    }
  PlanFootstepsResult_(const ContainerAllocator& _alloc)
    : result(_alloc)  {
  (void)_alloc;
    }



   typedef  ::jsk_footstep_msgs::FootstepArray_<ContainerAllocator>  _result_type;
  _result_type result;





  typedef boost::shared_ptr< ::jsk_footstep_msgs::PlanFootstepsResult_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::jsk_footstep_msgs::PlanFootstepsResult_<ContainerAllocator> const> ConstPtr;

}; // struct PlanFootstepsResult_

typedef ::jsk_footstep_msgs::PlanFootstepsResult_<std::allocator<void> > PlanFootstepsResult;

typedef boost::shared_ptr< ::jsk_footstep_msgs::PlanFootstepsResult > PlanFootstepsResultPtr;
typedef boost::shared_ptr< ::jsk_footstep_msgs::PlanFootstepsResult const> PlanFootstepsResultConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::jsk_footstep_msgs::PlanFootstepsResult_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::jsk_footstep_msgs::PlanFootstepsResult_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace jsk_footstep_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'actionlib_msgs': ['/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg'], 'jsk_footstep_msgs': ['/tmp/binarydeb/ros-kinetic-jsk-footstep-msgs-4.3.1/msg', '/tmp/binarydeb/ros-kinetic-jsk-footstep-msgs-4.3.1/obj-x86_64-linux-gnu/devel/share/jsk_footstep_msgs/msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::jsk_footstep_msgs::PlanFootstepsResult_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::jsk_footstep_msgs::PlanFootstepsResult_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::jsk_footstep_msgs::PlanFootstepsResult_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::jsk_footstep_msgs::PlanFootstepsResult_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::jsk_footstep_msgs::PlanFootstepsResult_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::jsk_footstep_msgs::PlanFootstepsResult_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::jsk_footstep_msgs::PlanFootstepsResult_<ContainerAllocator> >
{
  static const char* value()
  {
    return "951c674749b5b8b08d5eba2e3e62aedd";
  }

  static const char* value(const ::jsk_footstep_msgs::PlanFootstepsResult_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x951c674749b5b8b0ULL;
  static const uint64_t static_value2 = 0x8d5eba2e3e62aeddULL;
};

template<class ContainerAllocator>
struct DataType< ::jsk_footstep_msgs::PlanFootstepsResult_<ContainerAllocator> >
{
  static const char* value()
  {
    return "jsk_footstep_msgs/PlanFootstepsResult";
  }

  static const char* value(const ::jsk_footstep_msgs::PlanFootstepsResult_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::jsk_footstep_msgs::PlanFootstepsResult_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
# result\n\
FootstepArray result\n\
\n\
================================================================================\n\
MSG: jsk_footstep_msgs/FootstepArray\n\
Header header\n\
Footstep[] footsteps\n\
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
MSG: jsk_footstep_msgs/Footstep\n\
uint8 RIGHT=2\n\
uint8 LEFT=1\n\
\n\
# Constants to visualize progress\n\
uint8 REJECTED=3\n\
uint8 APPROVED=4\n\
\n\
## limb_indicator values\n\
uint8 LLEG=1\n\
uint8 RLEG=2\n\
uint8 LARM=5\n\
uint8 RARM=6\n\
\n\
uint8 leg ## value should be one of limb_indicator values.\n\
geometry_msgs/Pose pose ## 'pose' represents nominal pose. It may be an end-effector of limb.\n\
duration duration\n\
# optional parameters\n\
uint32 footstep_group\n\
geometry_msgs/Vector3 dimensions ## cube [length(x), width(y), height(z)]\n\
geometry_msgs/Vector3 offset     ## offset from pose to center of cube\n\
float32 swing_height\n\
float32 cost\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Pose\n\
# A representation of pose in free space, composed of position and orientation. \n\
Point position\n\
Quaternion orientation\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Point\n\
# This contains the position of a point in free space\n\
float64 x\n\
float64 y\n\
float64 z\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Quaternion\n\
# This represents an orientation in free space in quaternion form.\n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
float64 w\n\
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

  static const char* value(const ::jsk_footstep_msgs::PlanFootstepsResult_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::jsk_footstep_msgs::PlanFootstepsResult_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.result);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct PlanFootstepsResult_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::jsk_footstep_msgs::PlanFootstepsResult_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::jsk_footstep_msgs::PlanFootstepsResult_<ContainerAllocator>& v)
  {
    s << indent << "result: ";
    s << std::endl;
    Printer< ::jsk_footstep_msgs::FootstepArray_<ContainerAllocator> >::stream(s, indent + "  ", v.result);
  }
};

} // namespace message_operations
} // namespace ros

#endif // JSK_FOOTSTEP_MSGS_MESSAGE_PLANFOOTSTEPSRESULT_H

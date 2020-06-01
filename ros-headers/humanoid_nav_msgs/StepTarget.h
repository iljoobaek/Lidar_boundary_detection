// Generated by gencpp from file humanoid_nav_msgs/StepTarget.msg
// DO NOT EDIT!


#ifndef HUMANOID_NAV_MSGS_MESSAGE_STEPTARGET_H
#define HUMANOID_NAV_MSGS_MESSAGE_STEPTARGET_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <geometry_msgs/Pose2D.h>

namespace humanoid_nav_msgs
{
template <class ContainerAllocator>
struct StepTarget_
{
  typedef StepTarget_<ContainerAllocator> Type;

  StepTarget_()
    : pose()
    , leg(0)  {
    }
  StepTarget_(const ContainerAllocator& _alloc)
    : pose(_alloc)
    , leg(0)  {
  (void)_alloc;
    }



   typedef  ::geometry_msgs::Pose2D_<ContainerAllocator>  _pose_type;
  _pose_type pose;

   typedef uint8_t _leg_type;
  _leg_type leg;



  enum {
    right = 0u,
    left = 1u,
  };


  typedef boost::shared_ptr< ::humanoid_nav_msgs::StepTarget_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::humanoid_nav_msgs::StepTarget_<ContainerAllocator> const> ConstPtr;

}; // struct StepTarget_

typedef ::humanoid_nav_msgs::StepTarget_<std::allocator<void> > StepTarget;

typedef boost::shared_ptr< ::humanoid_nav_msgs::StepTarget > StepTargetPtr;
typedef boost::shared_ptr< ::humanoid_nav_msgs::StepTarget const> StepTargetConstPtr;

// constants requiring out of line definition

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::humanoid_nav_msgs::StepTarget_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::humanoid_nav_msgs::StepTarget_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace humanoid_nav_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'humanoid_nav_msgs': ['/tmp/binarydeb/ros-kinetic-humanoid-nav-msgs-0.3.0/msg', '/tmp/binarydeb/ros-kinetic-humanoid-nav-msgs-0.3.0/obj-x86_64-linux-gnu/devel/share/humanoid_nav_msgs/msg'], 'actionlib_msgs': ['/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::humanoid_nav_msgs::StepTarget_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::humanoid_nav_msgs::StepTarget_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::humanoid_nav_msgs::StepTarget_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::humanoid_nav_msgs::StepTarget_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::humanoid_nav_msgs::StepTarget_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::humanoid_nav_msgs::StepTarget_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::humanoid_nav_msgs::StepTarget_<ContainerAllocator> >
{
  static const char* value()
  {
    return "8ccf34ddb67039fbda0d9b2515ebb1ea";
  }

  static const char* value(const ::humanoid_nav_msgs::StepTarget_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x8ccf34ddb67039fbULL;
  static const uint64_t static_value2 = 0xda0d9b2515ebb1eaULL;
};

template<class ContainerAllocator>
struct DataType< ::humanoid_nav_msgs::StepTarget_<ContainerAllocator> >
{
  static const char* value()
  {
    return "humanoid_nav_msgs/StepTarget";
  }

  static const char* value(const ::humanoid_nav_msgs::StepTarget_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::humanoid_nav_msgs::StepTarget_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# Target for a single stepping motion of a humanoid's leg\n\
\n\
geometry_msgs/Pose2D pose   # step pose as relative offset to last leg\n\
uint8 leg                   # which leg to use (left/right, see below)\n\
\n\
uint8 right=0               # right leg constant\n\
uint8 left=1                # left leg constant\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Pose2D\n\
# Deprecated\n\
# Please use the full 3D pose.\n\
\n\
# In general our recommendation is to use a full 3D representation of everything and for 2D specific applications make the appropriate projections into the plane for their calculations but optimally will preserve the 3D information during processing.\n\
\n\
# If we have parallel copies of 2D datatypes every UI and other pipeline will end up needing to have dual interfaces to plot everything. And you will end up with not being able to use 3D tools for 2D use cases even if they're completely valid, as you'd have to reimplement it with different inputs and outputs. It's not particularly hard to plot the 2D pose or compute the yaw error for the Pose message and there are already tools and libraries that can do this for you.\n\
\n\
\n\
# This expresses a position and orientation on a 2D manifold.\n\
\n\
float64 x\n\
float64 y\n\
float64 theta\n\
";
  }

  static const char* value(const ::humanoid_nav_msgs::StepTarget_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::humanoid_nav_msgs::StepTarget_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.pose);
      stream.next(m.leg);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct StepTarget_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::humanoid_nav_msgs::StepTarget_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::humanoid_nav_msgs::StepTarget_<ContainerAllocator>& v)
  {
    s << indent << "pose: ";
    s << std::endl;
    Printer< ::geometry_msgs::Pose2D_<ContainerAllocator> >::stream(s, indent + "  ", v.pose);
    s << indent << "leg: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.leg);
  }
};

} // namespace message_operations
} // namespace ros

#endif // HUMANOID_NAV_MSGS_MESSAGE_STEPTARGET_H

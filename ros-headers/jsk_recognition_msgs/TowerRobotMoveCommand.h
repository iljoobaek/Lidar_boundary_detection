// Generated by gencpp from file jsk_recognition_msgs/TowerRobotMoveCommand.msg
// DO NOT EDIT!


#ifndef JSK_RECOGNITION_MSGS_MESSAGE_TOWERROBOTMOVECOMMAND_H
#define JSK_RECOGNITION_MSGS_MESSAGE_TOWERROBOTMOVECOMMAND_H

#include <ros/service_traits.h>


#include <jsk_recognition_msgs/TowerRobotMoveCommandRequest.h>
#include <jsk_recognition_msgs/TowerRobotMoveCommandResponse.h>


namespace jsk_recognition_msgs
{

struct TowerRobotMoveCommand
{

typedef TowerRobotMoveCommandRequest Request;
typedef TowerRobotMoveCommandResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct TowerRobotMoveCommand
} // namespace jsk_recognition_msgs


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::jsk_recognition_msgs::TowerRobotMoveCommand > {
  static const char* value()
  {
    return "aadba056bdce0494569ab50ecd2ec90c";
  }

  static const char* value(const ::jsk_recognition_msgs::TowerRobotMoveCommand&) { return value(); }
};

template<>
struct DataType< ::jsk_recognition_msgs::TowerRobotMoveCommand > {
  static const char* value()
  {
    return "jsk_recognition_msgs/TowerRobotMoveCommand";
  }

  static const char* value(const ::jsk_recognition_msgs::TowerRobotMoveCommand&) { return value(); }
};


// service_traits::MD5Sum< ::jsk_recognition_msgs::TowerRobotMoveCommandRequest> should match 
// service_traits::MD5Sum< ::jsk_recognition_msgs::TowerRobotMoveCommand > 
template<>
struct MD5Sum< ::jsk_recognition_msgs::TowerRobotMoveCommandRequest>
{
  static const char* value()
  {
    return MD5Sum< ::jsk_recognition_msgs::TowerRobotMoveCommand >::value();
  }
  static const char* value(const ::jsk_recognition_msgs::TowerRobotMoveCommandRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::jsk_recognition_msgs::TowerRobotMoveCommandRequest> should match 
// service_traits::DataType< ::jsk_recognition_msgs::TowerRobotMoveCommand > 
template<>
struct DataType< ::jsk_recognition_msgs::TowerRobotMoveCommandRequest>
{
  static const char* value()
  {
    return DataType< ::jsk_recognition_msgs::TowerRobotMoveCommand >::value();
  }
  static const char* value(const ::jsk_recognition_msgs::TowerRobotMoveCommandRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::jsk_recognition_msgs::TowerRobotMoveCommandResponse> should match 
// service_traits::MD5Sum< ::jsk_recognition_msgs::TowerRobotMoveCommand > 
template<>
struct MD5Sum< ::jsk_recognition_msgs::TowerRobotMoveCommandResponse>
{
  static const char* value()
  {
    return MD5Sum< ::jsk_recognition_msgs::TowerRobotMoveCommand >::value();
  }
  static const char* value(const ::jsk_recognition_msgs::TowerRobotMoveCommandResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::jsk_recognition_msgs::TowerRobotMoveCommandResponse> should match 
// service_traits::DataType< ::jsk_recognition_msgs::TowerRobotMoveCommand > 
template<>
struct DataType< ::jsk_recognition_msgs::TowerRobotMoveCommandResponse>
{
  static const char* value()
  {
    return DataType< ::jsk_recognition_msgs::TowerRobotMoveCommand >::value();
  }
  static const char* value(const ::jsk_recognition_msgs::TowerRobotMoveCommandResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // JSK_RECOGNITION_MSGS_MESSAGE_TOWERROBOTMOVECOMMAND_H

// Generated by gencpp from file jsk_recognition_msgs/SwitchTopic.msg
// DO NOT EDIT!


#ifndef JSK_RECOGNITION_MSGS_MESSAGE_SWITCHTOPIC_H
#define JSK_RECOGNITION_MSGS_MESSAGE_SWITCHTOPIC_H

#include <ros/service_traits.h>


#include <jsk_recognition_msgs/SwitchTopicRequest.h>
#include <jsk_recognition_msgs/SwitchTopicResponse.h>


namespace jsk_recognition_msgs
{

struct SwitchTopic
{

typedef SwitchTopicRequest Request;
typedef SwitchTopicResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct SwitchTopic
} // namespace jsk_recognition_msgs


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::jsk_recognition_msgs::SwitchTopic > {
  static const char* value()
  {
    return "e4a276b5a9b7b8fd97441d0fd991bdb9";
  }

  static const char* value(const ::jsk_recognition_msgs::SwitchTopic&) { return value(); }
};

template<>
struct DataType< ::jsk_recognition_msgs::SwitchTopic > {
  static const char* value()
  {
    return "jsk_recognition_msgs/SwitchTopic";
  }

  static const char* value(const ::jsk_recognition_msgs::SwitchTopic&) { return value(); }
};


// service_traits::MD5Sum< ::jsk_recognition_msgs::SwitchTopicRequest> should match 
// service_traits::MD5Sum< ::jsk_recognition_msgs::SwitchTopic > 
template<>
struct MD5Sum< ::jsk_recognition_msgs::SwitchTopicRequest>
{
  static const char* value()
  {
    return MD5Sum< ::jsk_recognition_msgs::SwitchTopic >::value();
  }
  static const char* value(const ::jsk_recognition_msgs::SwitchTopicRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::jsk_recognition_msgs::SwitchTopicRequest> should match 
// service_traits::DataType< ::jsk_recognition_msgs::SwitchTopic > 
template<>
struct DataType< ::jsk_recognition_msgs::SwitchTopicRequest>
{
  static const char* value()
  {
    return DataType< ::jsk_recognition_msgs::SwitchTopic >::value();
  }
  static const char* value(const ::jsk_recognition_msgs::SwitchTopicRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::jsk_recognition_msgs::SwitchTopicResponse> should match 
// service_traits::MD5Sum< ::jsk_recognition_msgs::SwitchTopic > 
template<>
struct MD5Sum< ::jsk_recognition_msgs::SwitchTopicResponse>
{
  static const char* value()
  {
    return MD5Sum< ::jsk_recognition_msgs::SwitchTopic >::value();
  }
  static const char* value(const ::jsk_recognition_msgs::SwitchTopicResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::jsk_recognition_msgs::SwitchTopicResponse> should match 
// service_traits::DataType< ::jsk_recognition_msgs::SwitchTopic > 
template<>
struct DataType< ::jsk_recognition_msgs::SwitchTopicResponse>
{
  static const char* value()
  {
    return DataType< ::jsk_recognition_msgs::SwitchTopic >::value();
  }
  static const char* value(const ::jsk_recognition_msgs::SwitchTopicResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // JSK_RECOGNITION_MSGS_MESSAGE_SWITCHTOPIC_H

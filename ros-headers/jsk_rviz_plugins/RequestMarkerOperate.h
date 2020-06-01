// Generated by gencpp from file jsk_rviz_plugins/RequestMarkerOperate.msg
// DO NOT EDIT!


#ifndef JSK_RVIZ_PLUGINS_MESSAGE_REQUESTMARKEROPERATE_H
#define JSK_RVIZ_PLUGINS_MESSAGE_REQUESTMARKEROPERATE_H

#include <ros/service_traits.h>


#include <jsk_rviz_plugins/RequestMarkerOperateRequest.h>
#include <jsk_rviz_plugins/RequestMarkerOperateResponse.h>


namespace jsk_rviz_plugins
{

struct RequestMarkerOperate
{

typedef RequestMarkerOperateRequest Request;
typedef RequestMarkerOperateResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct RequestMarkerOperate
} // namespace jsk_rviz_plugins


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::jsk_rviz_plugins::RequestMarkerOperate > {
  static const char* value()
  {
    return "5d5e6dca1cfed7e0be1a8c17221d0619";
  }

  static const char* value(const ::jsk_rviz_plugins::RequestMarkerOperate&) { return value(); }
};

template<>
struct DataType< ::jsk_rviz_plugins::RequestMarkerOperate > {
  static const char* value()
  {
    return "jsk_rviz_plugins/RequestMarkerOperate";
  }

  static const char* value(const ::jsk_rviz_plugins::RequestMarkerOperate&) { return value(); }
};


// service_traits::MD5Sum< ::jsk_rviz_plugins::RequestMarkerOperateRequest> should match 
// service_traits::MD5Sum< ::jsk_rviz_plugins::RequestMarkerOperate > 
template<>
struct MD5Sum< ::jsk_rviz_plugins::RequestMarkerOperateRequest>
{
  static const char* value()
  {
    return MD5Sum< ::jsk_rviz_plugins::RequestMarkerOperate >::value();
  }
  static const char* value(const ::jsk_rviz_plugins::RequestMarkerOperateRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::jsk_rviz_plugins::RequestMarkerOperateRequest> should match 
// service_traits::DataType< ::jsk_rviz_plugins::RequestMarkerOperate > 
template<>
struct DataType< ::jsk_rviz_plugins::RequestMarkerOperateRequest>
{
  static const char* value()
  {
    return DataType< ::jsk_rviz_plugins::RequestMarkerOperate >::value();
  }
  static const char* value(const ::jsk_rviz_plugins::RequestMarkerOperateRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::jsk_rviz_plugins::RequestMarkerOperateResponse> should match 
// service_traits::MD5Sum< ::jsk_rviz_plugins::RequestMarkerOperate > 
template<>
struct MD5Sum< ::jsk_rviz_plugins::RequestMarkerOperateResponse>
{
  static const char* value()
  {
    return MD5Sum< ::jsk_rviz_plugins::RequestMarkerOperate >::value();
  }
  static const char* value(const ::jsk_rviz_plugins::RequestMarkerOperateResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::jsk_rviz_plugins::RequestMarkerOperateResponse> should match 
// service_traits::DataType< ::jsk_rviz_plugins::RequestMarkerOperate > 
template<>
struct DataType< ::jsk_rviz_plugins::RequestMarkerOperateResponse>
{
  static const char* value()
  {
    return DataType< ::jsk_rviz_plugins::RequestMarkerOperate >::value();
  }
  static const char* value(const ::jsk_rviz_plugins::RequestMarkerOperateResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // JSK_RVIZ_PLUGINS_MESSAGE_REQUESTMARKEROPERATE_H

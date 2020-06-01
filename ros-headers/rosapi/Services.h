// Generated by gencpp from file rosapi/Services.msg
// DO NOT EDIT!


#ifndef ROSAPI_MESSAGE_SERVICES_H
#define ROSAPI_MESSAGE_SERVICES_H

#include <ros/service_traits.h>


#include <rosapi/ServicesRequest.h>
#include <rosapi/ServicesResponse.h>


namespace rosapi
{

struct Services
{

typedef ServicesRequest Request;
typedef ServicesResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct Services
} // namespace rosapi


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::rosapi::Services > {
  static const char* value()
  {
    return "e44a7e7bcb900acadbcc28b132378f0c";
  }

  static const char* value(const ::rosapi::Services&) { return value(); }
};

template<>
struct DataType< ::rosapi::Services > {
  static const char* value()
  {
    return "rosapi/Services";
  }

  static const char* value(const ::rosapi::Services&) { return value(); }
};


// service_traits::MD5Sum< ::rosapi::ServicesRequest> should match 
// service_traits::MD5Sum< ::rosapi::Services > 
template<>
struct MD5Sum< ::rosapi::ServicesRequest>
{
  static const char* value()
  {
    return MD5Sum< ::rosapi::Services >::value();
  }
  static const char* value(const ::rosapi::ServicesRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::rosapi::ServicesRequest> should match 
// service_traits::DataType< ::rosapi::Services > 
template<>
struct DataType< ::rosapi::ServicesRequest>
{
  static const char* value()
  {
    return DataType< ::rosapi::Services >::value();
  }
  static const char* value(const ::rosapi::ServicesRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::rosapi::ServicesResponse> should match 
// service_traits::MD5Sum< ::rosapi::Services > 
template<>
struct MD5Sum< ::rosapi::ServicesResponse>
{
  static const char* value()
  {
    return MD5Sum< ::rosapi::Services >::value();
  }
  static const char* value(const ::rosapi::ServicesResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::rosapi::ServicesResponse> should match 
// service_traits::DataType< ::rosapi::Services > 
template<>
struct DataType< ::rosapi::ServicesResponse>
{
  static const char* value()
  {
    return DataType< ::rosapi::Services >::value();
  }
  static const char* value(const ::rosapi::ServicesResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // ROSAPI_MESSAGE_SERVICES_H

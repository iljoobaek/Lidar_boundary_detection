// Generated by gencpp from file jsk_topic_tools/List.msg
// DO NOT EDIT!


#ifndef JSK_TOPIC_TOOLS_MESSAGE_LIST_H
#define JSK_TOPIC_TOOLS_MESSAGE_LIST_H

#include <ros/service_traits.h>


#include <jsk_topic_tools/ListRequest.h>
#include <jsk_topic_tools/ListResponse.h>


namespace jsk_topic_tools
{

struct List
{

typedef ListRequest Request;
typedef ListResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct List
} // namespace jsk_topic_tools


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::jsk_topic_tools::List > {
  static const char* value()
  {
    return "f88f7a076bf16dde010b2480af40dcdb";
  }

  static const char* value(const ::jsk_topic_tools::List&) { return value(); }
};

template<>
struct DataType< ::jsk_topic_tools::List > {
  static const char* value()
  {
    return "jsk_topic_tools/List";
  }

  static const char* value(const ::jsk_topic_tools::List&) { return value(); }
};


// service_traits::MD5Sum< ::jsk_topic_tools::ListRequest> should match 
// service_traits::MD5Sum< ::jsk_topic_tools::List > 
template<>
struct MD5Sum< ::jsk_topic_tools::ListRequest>
{
  static const char* value()
  {
    return MD5Sum< ::jsk_topic_tools::List >::value();
  }
  static const char* value(const ::jsk_topic_tools::ListRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::jsk_topic_tools::ListRequest> should match 
// service_traits::DataType< ::jsk_topic_tools::List > 
template<>
struct DataType< ::jsk_topic_tools::ListRequest>
{
  static const char* value()
  {
    return DataType< ::jsk_topic_tools::List >::value();
  }
  static const char* value(const ::jsk_topic_tools::ListRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::jsk_topic_tools::ListResponse> should match 
// service_traits::MD5Sum< ::jsk_topic_tools::List > 
template<>
struct MD5Sum< ::jsk_topic_tools::ListResponse>
{
  static const char* value()
  {
    return MD5Sum< ::jsk_topic_tools::List >::value();
  }
  static const char* value(const ::jsk_topic_tools::ListResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::jsk_topic_tools::ListResponse> should match 
// service_traits::DataType< ::jsk_topic_tools::List > 
template<>
struct DataType< ::jsk_topic_tools::ListResponse>
{
  static const char* value()
  {
    return DataType< ::jsk_topic_tools::List >::value();
  }
  static const char* value(const ::jsk_topic_tools::ListResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // JSK_TOPIC_TOOLS_MESSAGE_LIST_H

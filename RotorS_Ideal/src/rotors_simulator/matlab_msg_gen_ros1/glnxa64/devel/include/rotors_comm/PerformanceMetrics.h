// Generated by gencpp from file rotors_comm/PerformanceMetrics.msg
// DO NOT EDIT!


#ifndef ROTORS_COMM_MESSAGE_PERFORMANCEMETRICS_H
#define ROTORS_COMM_MESSAGE_PERFORMANCEMETRICS_H

#include <ros/service_traits.h>


#include <rotors_comm/PerformanceMetricsRequest.h>
#include <rotors_comm/PerformanceMetricsResponse.h>


namespace rotors_comm
{

struct PerformanceMetrics
{

typedef PerformanceMetricsRequest Request;
typedef PerformanceMetricsResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct PerformanceMetrics
} // namespace rotors_comm


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::rotors_comm::PerformanceMetrics > {
  static const char* value()
  {
    return "489ff84073d2b57991c40f5769f49311";
  }

  static const char* value(const ::rotors_comm::PerformanceMetrics&) { return value(); }
};

template<>
struct DataType< ::rotors_comm::PerformanceMetrics > {
  static const char* value()
  {
    return "rotors_comm/PerformanceMetrics";
  }

  static const char* value(const ::rotors_comm::PerformanceMetrics&) { return value(); }
};


// service_traits::MD5Sum< ::rotors_comm::PerformanceMetricsRequest> should match
// service_traits::MD5Sum< ::rotors_comm::PerformanceMetrics >
template<>
struct MD5Sum< ::rotors_comm::PerformanceMetricsRequest>
{
  static const char* value()
  {
    return MD5Sum< ::rotors_comm::PerformanceMetrics >::value();
  }
  static const char* value(const ::rotors_comm::PerformanceMetricsRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::rotors_comm::PerformanceMetricsRequest> should match
// service_traits::DataType< ::rotors_comm::PerformanceMetrics >
template<>
struct DataType< ::rotors_comm::PerformanceMetricsRequest>
{
  static const char* value()
  {
    return DataType< ::rotors_comm::PerformanceMetrics >::value();
  }
  static const char* value(const ::rotors_comm::PerformanceMetricsRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::rotors_comm::PerformanceMetricsResponse> should match
// service_traits::MD5Sum< ::rotors_comm::PerformanceMetrics >
template<>
struct MD5Sum< ::rotors_comm::PerformanceMetricsResponse>
{
  static const char* value()
  {
    return MD5Sum< ::rotors_comm::PerformanceMetrics >::value();
  }
  static const char* value(const ::rotors_comm::PerformanceMetricsResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::rotors_comm::PerformanceMetricsResponse> should match
// service_traits::DataType< ::rotors_comm::PerformanceMetrics >
template<>
struct DataType< ::rotors_comm::PerformanceMetricsResponse>
{
  static const char* value()
  {
    return DataType< ::rotors_comm::PerformanceMetrics >::value();
  }
  static const char* value(const ::rotors_comm::PerformanceMetricsResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // ROTORS_COMM_MESSAGE_PERFORMANCEMETRICS_H

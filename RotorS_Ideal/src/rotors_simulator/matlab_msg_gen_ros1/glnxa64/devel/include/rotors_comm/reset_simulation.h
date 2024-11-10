// Generated by gencpp from file rotors_comm/reset_simulation.msg
// DO NOT EDIT!


#ifndef ROTORS_COMM_MESSAGE_RESET_SIMULATION_H
#define ROTORS_COMM_MESSAGE_RESET_SIMULATION_H

#include <ros/service_traits.h>


#include <rotors_comm/reset_simulationRequest.h>
#include <rotors_comm/reset_simulationResponse.h>


namespace rotors_comm
{

struct reset_simulation
{

typedef reset_simulationRequest Request;
typedef reset_simulationResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct reset_simulation
} // namespace rotors_comm


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::rotors_comm::reset_simulation > {
  static const char* value()
  {
    return "358e233cde0c8a8bcfea4ce193f8fc15";
  }

  static const char* value(const ::rotors_comm::reset_simulation&) { return value(); }
};

template<>
struct DataType< ::rotors_comm::reset_simulation > {
  static const char* value()
  {
    return "rotors_comm/reset_simulation";
  }

  static const char* value(const ::rotors_comm::reset_simulation&) { return value(); }
};


// service_traits::MD5Sum< ::rotors_comm::reset_simulationRequest> should match
// service_traits::MD5Sum< ::rotors_comm::reset_simulation >
template<>
struct MD5Sum< ::rotors_comm::reset_simulationRequest>
{
  static const char* value()
  {
    return MD5Sum< ::rotors_comm::reset_simulation >::value();
  }
  static const char* value(const ::rotors_comm::reset_simulationRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::rotors_comm::reset_simulationRequest> should match
// service_traits::DataType< ::rotors_comm::reset_simulation >
template<>
struct DataType< ::rotors_comm::reset_simulationRequest>
{
  static const char* value()
  {
    return DataType< ::rotors_comm::reset_simulation >::value();
  }
  static const char* value(const ::rotors_comm::reset_simulationRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::rotors_comm::reset_simulationResponse> should match
// service_traits::MD5Sum< ::rotors_comm::reset_simulation >
template<>
struct MD5Sum< ::rotors_comm::reset_simulationResponse>
{
  static const char* value()
  {
    return MD5Sum< ::rotors_comm::reset_simulation >::value();
  }
  static const char* value(const ::rotors_comm::reset_simulationResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::rotors_comm::reset_simulationResponse> should match
// service_traits::DataType< ::rotors_comm::reset_simulation >
template<>
struct DataType< ::rotors_comm::reset_simulationResponse>
{
  static const char* value()
  {
    return DataType< ::rotors_comm::reset_simulation >::value();
  }
  static const char* value(const ::rotors_comm::reset_simulationResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // ROTORS_COMM_MESSAGE_RESET_SIMULATION_H
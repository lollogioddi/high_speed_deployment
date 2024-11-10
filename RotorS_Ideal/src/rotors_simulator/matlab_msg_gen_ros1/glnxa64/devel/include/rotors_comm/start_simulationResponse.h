// Generated by gencpp from file rotors_comm/start_simulationResponse.msg
// DO NOT EDIT!


#ifndef ROTORS_COMM_MESSAGE_START_SIMULATIONRESPONSE_H
#define ROTORS_COMM_MESSAGE_START_SIMULATIONRESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace rotors_comm
{
template <class ContainerAllocator>
struct start_simulationResponse_
{
  typedef start_simulationResponse_<ContainerAllocator> Type;

  start_simulationResponse_()
    : success(false)  {
    }
  start_simulationResponse_(const ContainerAllocator& _alloc)
    : success(false)  {
  (void)_alloc;
    }



   typedef uint8_t _success_type;
  _success_type success;





  typedef boost::shared_ptr< ::rotors_comm::start_simulationResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::rotors_comm::start_simulationResponse_<ContainerAllocator> const> ConstPtr;

}; // struct start_simulationResponse_

typedef ::rotors_comm::start_simulationResponse_<std::allocator<void> > start_simulationResponse;

typedef boost::shared_ptr< ::rotors_comm::start_simulationResponse > start_simulationResponsePtr;
typedef boost::shared_ptr< ::rotors_comm::start_simulationResponse const> start_simulationResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::rotors_comm::start_simulationResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::rotors_comm::start_simulationResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::rotors_comm::start_simulationResponse_<ContainerAllocator1> & lhs, const ::rotors_comm::start_simulationResponse_<ContainerAllocator2> & rhs)
{
  return lhs.success == rhs.success;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::rotors_comm::start_simulationResponse_<ContainerAllocator1> & lhs, const ::rotors_comm::start_simulationResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace rotors_comm

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::rotors_comm::start_simulationResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::rotors_comm::start_simulationResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::rotors_comm::start_simulationResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::rotors_comm::start_simulationResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::rotors_comm::start_simulationResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::rotors_comm::start_simulationResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::rotors_comm::start_simulationResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "358e233cde0c8a8bcfea4ce193f8fc15";
  }

  static const char* value(const ::rotors_comm::start_simulationResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x358e233cde0c8a8bULL;
  static const uint64_t static_value2 = 0xcfea4ce193f8fc15ULL;
};

template<class ContainerAllocator>
struct DataType< ::rotors_comm::start_simulationResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "rotors_comm/start_simulationResponse";
  }

  static const char* value(const ::rotors_comm::start_simulationResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::rotors_comm::start_simulationResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bool success\n"
"\n"
"\n"
;
  }

  static const char* value(const ::rotors_comm::start_simulationResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::rotors_comm::start_simulationResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.success);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct start_simulationResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::rotors_comm::start_simulationResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::rotors_comm::start_simulationResponse_<ContainerAllocator>& v)
  {
    s << indent << "success: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.success);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ROTORS_COMM_MESSAGE_START_SIMULATIONRESPONSE_H
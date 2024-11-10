// Copyright 2019-2021 The MathWorks, Inc.
// Common copy functions for rotors_comm/WindSpeed
#include "boost/date_time.hpp"
#include "boost/shared_array.hpp"
#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable : 4244)
#pragma warning(disable : 4265)
#pragma warning(disable : 4458)
#pragma warning(disable : 4100)
#pragma warning(disable : 4127)
#pragma warning(disable : 4267)
#pragma warning(disable : 4068)
#pragma warning(disable : 4245)
#else
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wunused-local-typedefs"
#pragma GCC diagnostic ignored "-Wredundant-decls"
#pragma GCC diagnostic ignored "-Wnon-virtual-dtor"
#pragma GCC diagnostic ignored "-Wdelete-non-virtual-dtor"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wunused-variable"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif //_MSC_VER
#include "ros/ros.h"
#include "rotors_comm/WindSpeed.h"
#include "visibility_control.h"
#include "MATLABROSMsgInterface.hpp"
#include "ROSPubSubTemplates.hpp"
class ROTORS_COMM_EXPORT rotors_comm_msg_WindSpeed_common : public MATLABROSMsgInterface<rotors_comm::WindSpeed> {
  public:
    virtual ~rotors_comm_msg_WindSpeed_common(){}
    virtual void copy_from_struct(rotors_comm::WindSpeed* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const rotors_comm::WindSpeed* msg, MultiLibLoader loader, size_t size = 1);
};
  void rotors_comm_msg_WindSpeed_common::copy_from_struct(rotors_comm::WindSpeed* msg, const matlab::data::Struct& arr,
               MultiLibLoader loader) {
    try {
        //header
        const matlab::data::StructArray header_arr = arr["Header"];
        auto msgClassPtr_header = getCommonObject<std_msgs::Header>("std_msgs_msg_Header_common",loader);
        msgClassPtr_header->copy_from_struct(&msg->header,header_arr[0],loader);
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Header' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Header' is wrong type; expected a struct.");
    }
    try {
        //velocity
        const matlab::data::StructArray velocity_arr = arr["Velocity"];
        auto msgClassPtr_velocity = getCommonObject<geometry_msgs::Vector3>("geometry_msgs_msg_Vector3_common",loader);
        msgClassPtr_velocity->copy_from_struct(&msg->velocity,velocity_arr[0],loader);
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Velocity' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Velocity' is wrong type; expected a struct.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T rotors_comm_msg_WindSpeed_common::get_arr(MDFactory_T& factory, const rotors_comm::WindSpeed* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType","Header","Velocity"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("rotors_comm/WindSpeed");
    // header
    auto currentElement_header = (msg + ctr)->header;
    auto msgClassPtr_header = getCommonObject<std_msgs::Header>("std_msgs_msg_Header_common",loader);
    outArray[ctr]["Header"] = msgClassPtr_header->get_arr(factory, &currentElement_header, loader);
    // velocity
    auto currentElement_velocity = (msg + ctr)->velocity;
    auto msgClassPtr_velocity = getCommonObject<geometry_msgs::Vector3>("geometry_msgs_msg_Vector3_common",loader);
    outArray[ctr]["Velocity"] = msgClassPtr_velocity->get_arr(factory, &currentElement_velocity, loader);
    }
    return std::move(outArray);
  } 
class ROTORS_COMM_EXPORT rotors_comm_WindSpeed_message : public ROSMsgElementInterfaceFactory {
  public:
    virtual ~rotors_comm_WindSpeed_message(){}
    virtual std::shared_ptr<MATLABPublisherInterface> generatePublisherInterface(ElementType type);
    virtual std::shared_ptr<MATLABSubscriberInterface> generateSubscriberInterface(ElementType type);
    virtual std::shared_ptr<MATLABRosbagWriterInterface> generateRosbagWriterInterface(ElementType type);
};  
  std::shared_ptr<MATLABPublisherInterface> 
          rotors_comm_WindSpeed_message::generatePublisherInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSPublisherImpl<rotors_comm::WindSpeed,rotors_comm_msg_WindSpeed_common>>();
  }
  std::shared_ptr<MATLABSubscriberInterface> 
         rotors_comm_WindSpeed_message::generateSubscriberInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSSubscriberImpl<rotors_comm::WindSpeed,rotors_comm::WindSpeed::ConstPtr,rotors_comm_msg_WindSpeed_common>>();
  }
#include "ROSbagTemplates.hpp" 
  std::shared_ptr<MATLABRosbagWriterInterface>
         rotors_comm_WindSpeed_message::generateRosbagWriterInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSBagWriterImpl<rotors_comm::WindSpeed,rotors_comm_msg_WindSpeed_common>>();
  }
#include "register_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(rotors_comm_msg_WindSpeed_common, MATLABROSMsgInterface<rotors_comm::WindSpeed>)
CLASS_LOADER_REGISTER_CLASS(rotors_comm_WindSpeed_message, ROSMsgElementInterfaceFactory)
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif //_MSC_VER
//gen-1
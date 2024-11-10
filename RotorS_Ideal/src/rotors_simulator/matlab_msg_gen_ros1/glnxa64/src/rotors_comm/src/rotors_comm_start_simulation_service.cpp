// Copyright 2019-2021 The MathWorks, Inc.
// Common copy functions for rotors_comm/start_simulationRequest
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
#include "rotors_comm/start_simulation.h"
#include "visibility_control.h"
#include "ROSPubSubTemplates.hpp"
#include "ROSServiceTemplates.hpp"
class ROTORS_COMM_EXPORT rotors_comm_msg_start_simulationRequest_common : public MATLABROSMsgInterface<rotors_comm::start_simulation::Request> {
  public:
    virtual ~rotors_comm_msg_start_simulationRequest_common(){}
    virtual void copy_from_struct(rotors_comm::start_simulation::Request* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const rotors_comm::start_simulation::Request* msg, MultiLibLoader loader, size_t size = 1);
};
  void rotors_comm_msg_start_simulationRequest_common::copy_from_struct(rotors_comm::start_simulation::Request* msg, const matlab::data::Struct& arr,
               MultiLibLoader loader) {
  }
  //----------------------------------------------------------------------------
  MDArray_T rotors_comm_msg_start_simulationRequest_common::get_arr(MDFactory_T& factory, const rotors_comm::start_simulation::Request* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("rotors_comm/start_simulationRequest");
    }
    return std::move(outArray);
  }
class ROTORS_COMM_EXPORT rotors_comm_msg_start_simulationResponse_common : public MATLABROSMsgInterface<rotors_comm::start_simulation::Response> {
  public:
    virtual ~rotors_comm_msg_start_simulationResponse_common(){}
    virtual void copy_from_struct(rotors_comm::start_simulation::Response* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const rotors_comm::start_simulation::Response* msg, MultiLibLoader loader, size_t size = 1);
};
  void rotors_comm_msg_start_simulationResponse_common::copy_from_struct(rotors_comm::start_simulation::Response* msg, const matlab::data::Struct& arr,
               MultiLibLoader loader) {
    try {
        //success
        const matlab::data::TypedArray<bool> success_arr = arr["Success"];
        msg->success = success_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Success' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Success' is wrong type; expected a logical.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T rotors_comm_msg_start_simulationResponse_common::get_arr(MDFactory_T& factory, const rotors_comm::start_simulation::Response* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType","Success"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("rotors_comm/start_simulationResponse");
    // success
    auto currentElement_success = (msg + ctr)->success;
    outArray[ctr]["Success"] = factory.createScalar(static_cast<bool>(currentElement_success));
    }
    return std::move(outArray);
  } 
class ROTORS_COMM_EXPORT rotors_comm_start_simulation_service : public ROSMsgElementInterfaceFactory {
  public:
    virtual ~rotors_comm_start_simulation_service(){}
    virtual std::shared_ptr<MATLABPublisherInterface> generatePublisherInterface(ElementType type);
    virtual std::shared_ptr<MATLABSubscriberInterface> generateSubscriberInterface(ElementType type);
    virtual std::shared_ptr<MATLABRosbagWriterInterface> generateRosbagWriterInterface(ElementType type);
    virtual std::shared_ptr<MATLABSvcServerInterface> generateSvcServerInterface();
    virtual std::shared_ptr<MATLABSvcClientInterface> generateSvcClientInterface();
};  
  std::shared_ptr<MATLABPublisherInterface> 
          rotors_comm_start_simulation_service::generatePublisherInterface(ElementType type){
    std::shared_ptr<MATLABPublisherInterface> ptr;
    if(type == eRequest){
        ptr = std::make_shared<ROSPublisherImpl<rotors_comm::start_simulation::Request,rotors_comm_msg_start_simulationRequest_common>>();
    }else if(type == eResponse){
        ptr = std::make_shared<ROSPublisherImpl<rotors_comm::start_simulation::Response,rotors_comm_msg_start_simulationResponse_common>>();
    }else{
        throw std::invalid_argument("Wrong input, Expected 'Request' or 'Response'");
    }
    return ptr;
  }
  std::shared_ptr<MATLABSubscriberInterface> 
          rotors_comm_start_simulation_service::generateSubscriberInterface(ElementType type){
    std::shared_ptr<MATLABSubscriberInterface> ptr;
    if(type == eRequest){
        ptr = std::make_shared<ROSSubscriberImpl<rotors_comm::start_simulation::Request,rotors_comm::start_simulation::Request::ConstPtr,rotors_comm_msg_start_simulationRequest_common>>();
    }else if(type == eResponse){
        ptr = std::make_shared<ROSSubscriberImpl<rotors_comm::start_simulation::Response,rotors_comm::start_simulation::Response::ConstPtr,rotors_comm_msg_start_simulationResponse_common>>();
    }else{
        throw std::invalid_argument("Wrong input, Expected 'Request' or 'Response'");
    }
    return ptr;
  }
  std::shared_ptr<MATLABSvcServerInterface> 
          rotors_comm_start_simulation_service::generateSvcServerInterface(){
    return std::make_shared<ROSSvcServerImpl<rotors_comm::start_simulation::Request,rotors_comm::start_simulation::Response,rotors_comm_msg_start_simulationRequest_common,rotors_comm_msg_start_simulationResponse_common>>();
  }
  std::shared_ptr<MATLABSvcClientInterface> 
          rotors_comm_start_simulation_service::generateSvcClientInterface(){
    return std::make_shared<ROSSvcClientImpl<rotors_comm::start_simulation,rotors_comm::start_simulation::Request,rotors_comm::start_simulation::Response,rotors_comm_msg_start_simulationRequest_common,rotors_comm_msg_start_simulationResponse_common>>();
  }
#include "ROSbagTemplates.hpp" 
  std::shared_ptr<MATLABRosbagWriterInterface> 
          rotors_comm_start_simulation_service::generateRosbagWriterInterface(ElementType type){
    std::shared_ptr<MATLABRosbagWriterInterface> ptr;
    if(type == eRequest){
        ptr = std::make_shared<ROSBagWriterImpl<rotors_comm::start_simulationRequest,rotors_comm_msg_start_simulationRequest_common>>();
    }else if(type == eResponse){
        ptr = std::make_shared<ROSBagWriterImpl<rotors_comm::start_simulationResponse,rotors_comm_msg_start_simulationResponse_common>>();
    }else{
        throw std::invalid_argument("Wrong input, Expected 'Request' or 'Response'");
    }
    return ptr;
  }
#include "register_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(rotors_comm_msg_start_simulationRequest_common, MATLABROSMsgInterface<rotors_comm::start_simulationRequest>)
CLASS_LOADER_REGISTER_CLASS(rotors_comm_msg_start_simulationResponse_common, MATLABROSMsgInterface<rotors_comm::start_simulationResponse>)
CLASS_LOADER_REGISTER_CLASS(rotors_comm_start_simulation_service, ROSMsgElementInterfaceFactory)
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif //_MSC_VER
//gen-1

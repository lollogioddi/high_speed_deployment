// Copyright 2019-2021 The MathWorks, Inc.
// Common copy functions for rotors_comm/PerformanceMetricsRequest
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
#include "rotors_comm/PerformanceMetrics.h"
#include "visibility_control.h"
#include "ROSPubSubTemplates.hpp"
#include "ROSServiceTemplates.hpp"
class ROTORS_COMM_EXPORT rotors_comm_msg_PerformanceMetricsRequest_common : public MATLABROSMsgInterface<rotors_comm::PerformanceMetrics::Request> {
  public:
    virtual ~rotors_comm_msg_PerformanceMetricsRequest_common(){}
    virtual void copy_from_struct(rotors_comm::PerformanceMetrics::Request* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const rotors_comm::PerformanceMetrics::Request* msg, MultiLibLoader loader, size_t size = 1);
};
  void rotors_comm_msg_PerformanceMetricsRequest_common::copy_from_struct(rotors_comm::PerformanceMetrics::Request* msg, const matlab::data::Struct& arr,
               MultiLibLoader loader) {
  }
  //----------------------------------------------------------------------------
  MDArray_T rotors_comm_msg_PerformanceMetricsRequest_common::get_arr(MDFactory_T& factory, const rotors_comm::PerformanceMetrics::Request* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("rotors_comm/PerformanceMetricsRequest");
    }
    return std::move(outArray);
  }
class ROTORS_COMM_EXPORT rotors_comm_msg_PerformanceMetricsResponse_common : public MATLABROSMsgInterface<rotors_comm::PerformanceMetrics::Response> {
  public:
    virtual ~rotors_comm_msg_PerformanceMetricsResponse_common(){}
    virtual void copy_from_struct(rotors_comm::PerformanceMetrics::Response* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const rotors_comm::PerformanceMetrics::Response* msg, MultiLibLoader loader, size_t size = 1);
};
  void rotors_comm_msg_PerformanceMetricsResponse_common::copy_from_struct(rotors_comm::PerformanceMetrics::Response* msg, const matlab::data::Struct& arr,
               MultiLibLoader loader) {
    try {
        //overshoot1
        const matlab::data::TypedArray<double> overshoot1_arr = arr["Overshoot1"];
        msg->overshoot1 = overshoot1_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Overshoot1' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Overshoot1' is wrong type; expected a double.");
    }
    try {
        //overshoot2
        const matlab::data::TypedArray<double> overshoot2_arr = arr["Overshoot2"];
        msg->overshoot2 = overshoot2_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Overshoot2' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Overshoot2' is wrong type; expected a double.");
    }
    try {
        //overshoot3
        const matlab::data::TypedArray<double> overshoot3_arr = arr["Overshoot3"];
        msg->overshoot3 = overshoot3_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Overshoot3' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Overshoot3' is wrong type; expected a double.");
    }
    try {
        //overshoot4
        const matlab::data::TypedArray<double> overshoot4_arr = arr["Overshoot4"];
        msg->overshoot4 = overshoot4_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Overshoot4' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Overshoot4' is wrong type; expected a double.");
    }
    try {
        //overshoot5
        const matlab::data::TypedArray<double> overshoot5_arr = arr["Overshoot5"];
        msg->overshoot5 = overshoot5_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Overshoot5' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Overshoot5' is wrong type; expected a double.");
    }
    try {
        //overshoot6
        const matlab::data::TypedArray<double> overshoot6_arr = arr["Overshoot6"];
        msg->overshoot6 = overshoot6_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Overshoot6' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Overshoot6' is wrong type; expected a double.");
    }
    try {
        //settling_time1
        const matlab::data::TypedArray<double> settling_time1_arr = arr["SettlingTime1"];
        msg->settling_time1 = settling_time1_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'SettlingTime1' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'SettlingTime1' is wrong type; expected a double.");
    }
    try {
        //settling_time2
        const matlab::data::TypedArray<double> settling_time2_arr = arr["SettlingTime2"];
        msg->settling_time2 = settling_time2_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'SettlingTime2' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'SettlingTime2' is wrong type; expected a double.");
    }
    try {
        //settling_time3
        const matlab::data::TypedArray<double> settling_time3_arr = arr["SettlingTime3"];
        msg->settling_time3 = settling_time3_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'SettlingTime3' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'SettlingTime3' is wrong type; expected a double.");
    }
    try {
        //settling_time4
        const matlab::data::TypedArray<double> settling_time4_arr = arr["SettlingTime4"];
        msg->settling_time4 = settling_time4_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'SettlingTime4' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'SettlingTime4' is wrong type; expected a double.");
    }
    try {
        //settling_time5
        const matlab::data::TypedArray<double> settling_time5_arr = arr["SettlingTime5"];
        msg->settling_time5 = settling_time5_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'SettlingTime5' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'SettlingTime5' is wrong type; expected a double.");
    }
    try {
        //settling_time6
        const matlab::data::TypedArray<double> settling_time6_arr = arr["SettlingTime6"];
        msg->settling_time6 = settling_time6_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'SettlingTime6' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'SettlingTime6' is wrong type; expected a double.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T rotors_comm_msg_PerformanceMetricsResponse_common::get_arr(MDFactory_T& factory, const rotors_comm::PerformanceMetrics::Response* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType","Overshoot1","Overshoot2","Overshoot3","Overshoot4","Overshoot5","Overshoot6","SettlingTime1","SettlingTime2","SettlingTime3","SettlingTime4","SettlingTime5","SettlingTime6"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("rotors_comm/PerformanceMetricsResponse");
    // overshoot1
    auto currentElement_overshoot1 = (msg + ctr)->overshoot1;
    outArray[ctr]["Overshoot1"] = factory.createScalar(currentElement_overshoot1);
    // overshoot2
    auto currentElement_overshoot2 = (msg + ctr)->overshoot2;
    outArray[ctr]["Overshoot2"] = factory.createScalar(currentElement_overshoot2);
    // overshoot3
    auto currentElement_overshoot3 = (msg + ctr)->overshoot3;
    outArray[ctr]["Overshoot3"] = factory.createScalar(currentElement_overshoot3);
    // overshoot4
    auto currentElement_overshoot4 = (msg + ctr)->overshoot4;
    outArray[ctr]["Overshoot4"] = factory.createScalar(currentElement_overshoot4);
    // overshoot5
    auto currentElement_overshoot5 = (msg + ctr)->overshoot5;
    outArray[ctr]["Overshoot5"] = factory.createScalar(currentElement_overshoot5);
    // overshoot6
    auto currentElement_overshoot6 = (msg + ctr)->overshoot6;
    outArray[ctr]["Overshoot6"] = factory.createScalar(currentElement_overshoot6);
    // settling_time1
    auto currentElement_settling_time1 = (msg + ctr)->settling_time1;
    outArray[ctr]["SettlingTime1"] = factory.createScalar(currentElement_settling_time1);
    // settling_time2
    auto currentElement_settling_time2 = (msg + ctr)->settling_time2;
    outArray[ctr]["SettlingTime2"] = factory.createScalar(currentElement_settling_time2);
    // settling_time3
    auto currentElement_settling_time3 = (msg + ctr)->settling_time3;
    outArray[ctr]["SettlingTime3"] = factory.createScalar(currentElement_settling_time3);
    // settling_time4
    auto currentElement_settling_time4 = (msg + ctr)->settling_time4;
    outArray[ctr]["SettlingTime4"] = factory.createScalar(currentElement_settling_time4);
    // settling_time5
    auto currentElement_settling_time5 = (msg + ctr)->settling_time5;
    outArray[ctr]["SettlingTime5"] = factory.createScalar(currentElement_settling_time5);
    // settling_time6
    auto currentElement_settling_time6 = (msg + ctr)->settling_time6;
    outArray[ctr]["SettlingTime6"] = factory.createScalar(currentElement_settling_time6);
    }
    return std::move(outArray);
  } 
class ROTORS_COMM_EXPORT rotors_comm_PerformanceMetrics_service : public ROSMsgElementInterfaceFactory {
  public:
    virtual ~rotors_comm_PerformanceMetrics_service(){}
    virtual std::shared_ptr<MATLABPublisherInterface> generatePublisherInterface(ElementType type);
    virtual std::shared_ptr<MATLABSubscriberInterface> generateSubscriberInterface(ElementType type);
    virtual std::shared_ptr<MATLABRosbagWriterInterface> generateRosbagWriterInterface(ElementType type);
    virtual std::shared_ptr<MATLABSvcServerInterface> generateSvcServerInterface();
    virtual std::shared_ptr<MATLABSvcClientInterface> generateSvcClientInterface();
};  
  std::shared_ptr<MATLABPublisherInterface> 
          rotors_comm_PerformanceMetrics_service::generatePublisherInterface(ElementType type){
    std::shared_ptr<MATLABPublisherInterface> ptr;
    if(type == eRequest){
        ptr = std::make_shared<ROSPublisherImpl<rotors_comm::PerformanceMetrics::Request,rotors_comm_msg_PerformanceMetricsRequest_common>>();
    }else if(type == eResponse){
        ptr = std::make_shared<ROSPublisherImpl<rotors_comm::PerformanceMetrics::Response,rotors_comm_msg_PerformanceMetricsResponse_common>>();
    }else{
        throw std::invalid_argument("Wrong input, Expected 'Request' or 'Response'");
    }
    return ptr;
  }
  std::shared_ptr<MATLABSubscriberInterface> 
          rotors_comm_PerformanceMetrics_service::generateSubscriberInterface(ElementType type){
    std::shared_ptr<MATLABSubscriberInterface> ptr;
    if(type == eRequest){
        ptr = std::make_shared<ROSSubscriberImpl<rotors_comm::PerformanceMetrics::Request,rotors_comm::PerformanceMetrics::Request::ConstPtr,rotors_comm_msg_PerformanceMetricsRequest_common>>();
    }else if(type == eResponse){
        ptr = std::make_shared<ROSSubscriberImpl<rotors_comm::PerformanceMetrics::Response,rotors_comm::PerformanceMetrics::Response::ConstPtr,rotors_comm_msg_PerformanceMetricsResponse_common>>();
    }else{
        throw std::invalid_argument("Wrong input, Expected 'Request' or 'Response'");
    }
    return ptr;
  }
  std::shared_ptr<MATLABSvcServerInterface> 
          rotors_comm_PerformanceMetrics_service::generateSvcServerInterface(){
    return std::make_shared<ROSSvcServerImpl<rotors_comm::PerformanceMetrics::Request,rotors_comm::PerformanceMetrics::Response,rotors_comm_msg_PerformanceMetricsRequest_common,rotors_comm_msg_PerformanceMetricsResponse_common>>();
  }
  std::shared_ptr<MATLABSvcClientInterface> 
          rotors_comm_PerformanceMetrics_service::generateSvcClientInterface(){
    return std::make_shared<ROSSvcClientImpl<rotors_comm::PerformanceMetrics,rotors_comm::PerformanceMetrics::Request,rotors_comm::PerformanceMetrics::Response,rotors_comm_msg_PerformanceMetricsRequest_common,rotors_comm_msg_PerformanceMetricsResponse_common>>();
  }
#include "ROSbagTemplates.hpp" 
  std::shared_ptr<MATLABRosbagWriterInterface> 
          rotors_comm_PerformanceMetrics_service::generateRosbagWriterInterface(ElementType type){
    std::shared_ptr<MATLABRosbagWriterInterface> ptr;
    if(type == eRequest){
        ptr = std::make_shared<ROSBagWriterImpl<rotors_comm::PerformanceMetricsRequest,rotors_comm_msg_PerformanceMetricsRequest_common>>();
    }else if(type == eResponse){
        ptr = std::make_shared<ROSBagWriterImpl<rotors_comm::PerformanceMetricsResponse,rotors_comm_msg_PerformanceMetricsResponse_common>>();
    }else{
        throw std::invalid_argument("Wrong input, Expected 'Request' or 'Response'");
    }
    return ptr;
  }
#include "register_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(rotors_comm_msg_PerformanceMetricsRequest_common, MATLABROSMsgInterface<rotors_comm::PerformanceMetricsRequest>)
CLASS_LOADER_REGISTER_CLASS(rotors_comm_msg_PerformanceMetricsResponse_common, MATLABROSMsgInterface<rotors_comm::PerformanceMetricsResponse>)
CLASS_LOADER_REGISTER_CLASS(rotors_comm_PerformanceMetrics_service, ROSMsgElementInterfaceFactory)
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif //_MSC_VER
//gen-1

// Copyright 2019-2021 The MathWorks, Inc.
// Common copy functions for rotors_comm/OctomapRequest
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
#include "rotors_comm/Octomap.h"
#include "visibility_control.h"
#include "ROSPubSubTemplates.hpp"
#include "ROSServiceTemplates.hpp"
class ROTORS_COMM_EXPORT rotors_comm_msg_OctomapRequest_common : public MATLABROSMsgInterface<rotors_comm::Octomap::Request> {
  public:
    virtual ~rotors_comm_msg_OctomapRequest_common(){}
    virtual void copy_from_struct(rotors_comm::Octomap::Request* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const rotors_comm::Octomap::Request* msg, MultiLibLoader loader, size_t size = 1);
};
  void rotors_comm_msg_OctomapRequest_common::copy_from_struct(rotors_comm::Octomap::Request* msg, const matlab::data::Struct& arr,
               MultiLibLoader loader) {
    try {
        //bounding_box_origin
        const matlab::data::StructArray bounding_box_origin_arr = arr["BoundingBoxOrigin"];
        auto msgClassPtr_bounding_box_origin = getCommonObject<geometry_msgs::Point>("geometry_msgs_msg_Point_common",loader);
        msgClassPtr_bounding_box_origin->copy_from_struct(&msg->bounding_box_origin,bounding_box_origin_arr[0],loader);
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'BoundingBoxOrigin' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'BoundingBoxOrigin' is wrong type; expected a struct.");
    }
    try {
        //bounding_box_lengths
        const matlab::data::StructArray bounding_box_lengths_arr = arr["BoundingBoxLengths"];
        auto msgClassPtr_bounding_box_lengths = getCommonObject<geometry_msgs::Point>("geometry_msgs_msg_Point_common",loader);
        msgClassPtr_bounding_box_lengths->copy_from_struct(&msg->bounding_box_lengths,bounding_box_lengths_arr[0],loader);
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'BoundingBoxLengths' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'BoundingBoxLengths' is wrong type; expected a struct.");
    }
    try {
        //leaf_size
        const matlab::data::TypedArray<double> leaf_size_arr = arr["LeafSize"];
        msg->leaf_size = leaf_size_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'LeafSize' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'LeafSize' is wrong type; expected a double.");
    }
    try {
        //publish_octomap
        const matlab::data::TypedArray<bool> publish_octomap_arr = arr["PublishOctomap"];
        msg->publish_octomap = publish_octomap_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'PublishOctomap' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'PublishOctomap' is wrong type; expected a logical.");
    }
    try {
        //filename
        const matlab::data::CharArray filename_arr = arr["Filename"];
        msg->filename = filename_arr.toAscii();
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Filename' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Filename' is wrong type; expected a string.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T rotors_comm_msg_OctomapRequest_common::get_arr(MDFactory_T& factory, const rotors_comm::Octomap::Request* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType","BoundingBoxOrigin","BoundingBoxLengths","LeafSize","PublishOctomap","Filename"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("rotors_comm/OctomapRequest");
    // bounding_box_origin
    auto currentElement_bounding_box_origin = (msg + ctr)->bounding_box_origin;
    auto msgClassPtr_bounding_box_origin = getCommonObject<geometry_msgs::Point>("geometry_msgs_msg_Point_common",loader);
    outArray[ctr]["BoundingBoxOrigin"] = msgClassPtr_bounding_box_origin->get_arr(factory, &currentElement_bounding_box_origin, loader);
    // bounding_box_lengths
    auto currentElement_bounding_box_lengths = (msg + ctr)->bounding_box_lengths;
    auto msgClassPtr_bounding_box_lengths = getCommonObject<geometry_msgs::Point>("geometry_msgs_msg_Point_common",loader);
    outArray[ctr]["BoundingBoxLengths"] = msgClassPtr_bounding_box_lengths->get_arr(factory, &currentElement_bounding_box_lengths, loader);
    // leaf_size
    auto currentElement_leaf_size = (msg + ctr)->leaf_size;
    outArray[ctr]["LeafSize"] = factory.createScalar(currentElement_leaf_size);
    // publish_octomap
    auto currentElement_publish_octomap = (msg + ctr)->publish_octomap;
    outArray[ctr]["PublishOctomap"] = factory.createScalar(static_cast<bool>(currentElement_publish_octomap));
    // filename
    auto currentElement_filename = (msg + ctr)->filename;
    outArray[ctr]["Filename"] = factory.createCharArray(currentElement_filename);
    }
    return std::move(outArray);
  }
class ROTORS_COMM_EXPORT rotors_comm_msg_OctomapResponse_common : public MATLABROSMsgInterface<rotors_comm::Octomap::Response> {
  public:
    virtual ~rotors_comm_msg_OctomapResponse_common(){}
    virtual void copy_from_struct(rotors_comm::Octomap::Response* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const rotors_comm::Octomap::Response* msg, MultiLibLoader loader, size_t size = 1);
};
  void rotors_comm_msg_OctomapResponse_common::copy_from_struct(rotors_comm::Octomap::Response* msg, const matlab::data::Struct& arr,
               MultiLibLoader loader) {
    try {
        //map
        const matlab::data::StructArray map_arr = arr["Map"];
        auto msgClassPtr_map = getCommonObject<octomap_msgs::Octomap>("octomap_msgs_msg_Octomap_common",loader);
        msgClassPtr_map->copy_from_struct(&msg->map,map_arr[0],loader);
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Map' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Map' is wrong type; expected a struct.");
    }
    try {
        //origin_latitude
        const matlab::data::TypedArray<double> origin_latitude_arr = arr["OriginLatitude"];
        msg->origin_latitude = origin_latitude_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'OriginLatitude' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'OriginLatitude' is wrong type; expected a double.");
    }
    try {
        //origin_longitude
        const matlab::data::TypedArray<double> origin_longitude_arr = arr["OriginLongitude"];
        msg->origin_longitude = origin_longitude_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'OriginLongitude' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'OriginLongitude' is wrong type; expected a double.");
    }
    try {
        //origin_altitude
        const matlab::data::TypedArray<double> origin_altitude_arr = arr["OriginAltitude"];
        msg->origin_altitude = origin_altitude_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'OriginAltitude' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'OriginAltitude' is wrong type; expected a double.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T rotors_comm_msg_OctomapResponse_common::get_arr(MDFactory_T& factory, const rotors_comm::Octomap::Response* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType","Map","OriginLatitude","OriginLongitude","OriginAltitude"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("rotors_comm/OctomapResponse");
    // map
    auto currentElement_map = (msg + ctr)->map;
    auto msgClassPtr_map = getCommonObject<octomap_msgs::Octomap>("octomap_msgs_msg_Octomap_common",loader);
    outArray[ctr]["Map"] = msgClassPtr_map->get_arr(factory, &currentElement_map, loader);
    // origin_latitude
    auto currentElement_origin_latitude = (msg + ctr)->origin_latitude;
    outArray[ctr]["OriginLatitude"] = factory.createScalar(currentElement_origin_latitude);
    // origin_longitude
    auto currentElement_origin_longitude = (msg + ctr)->origin_longitude;
    outArray[ctr]["OriginLongitude"] = factory.createScalar(currentElement_origin_longitude);
    // origin_altitude
    auto currentElement_origin_altitude = (msg + ctr)->origin_altitude;
    outArray[ctr]["OriginAltitude"] = factory.createScalar(currentElement_origin_altitude);
    }
    return std::move(outArray);
  } 
class ROTORS_COMM_EXPORT rotors_comm_Octomap_service : public ROSMsgElementInterfaceFactory {
  public:
    virtual ~rotors_comm_Octomap_service(){}
    virtual std::shared_ptr<MATLABPublisherInterface> generatePublisherInterface(ElementType type);
    virtual std::shared_ptr<MATLABSubscriberInterface> generateSubscriberInterface(ElementType type);
    virtual std::shared_ptr<MATLABRosbagWriterInterface> generateRosbagWriterInterface(ElementType type);
    virtual std::shared_ptr<MATLABSvcServerInterface> generateSvcServerInterface();
    virtual std::shared_ptr<MATLABSvcClientInterface> generateSvcClientInterface();
};  
  std::shared_ptr<MATLABPublisherInterface> 
          rotors_comm_Octomap_service::generatePublisherInterface(ElementType type){
    std::shared_ptr<MATLABPublisherInterface> ptr;
    if(type == eRequest){
        ptr = std::make_shared<ROSPublisherImpl<rotors_comm::Octomap::Request,rotors_comm_msg_OctomapRequest_common>>();
    }else if(type == eResponse){
        ptr = std::make_shared<ROSPublisherImpl<rotors_comm::Octomap::Response,rotors_comm_msg_OctomapResponse_common>>();
    }else{
        throw std::invalid_argument("Wrong input, Expected 'Request' or 'Response'");
    }
    return ptr;
  }
  std::shared_ptr<MATLABSubscriberInterface> 
          rotors_comm_Octomap_service::generateSubscriberInterface(ElementType type){
    std::shared_ptr<MATLABSubscriberInterface> ptr;
    if(type == eRequest){
        ptr = std::make_shared<ROSSubscriberImpl<rotors_comm::Octomap::Request,rotors_comm::Octomap::Request::ConstPtr,rotors_comm_msg_OctomapRequest_common>>();
    }else if(type == eResponse){
        ptr = std::make_shared<ROSSubscriberImpl<rotors_comm::Octomap::Response,rotors_comm::Octomap::Response::ConstPtr,rotors_comm_msg_OctomapResponse_common>>();
    }else{
        throw std::invalid_argument("Wrong input, Expected 'Request' or 'Response'");
    }
    return ptr;
  }
  std::shared_ptr<MATLABSvcServerInterface> 
          rotors_comm_Octomap_service::generateSvcServerInterface(){
    return std::make_shared<ROSSvcServerImpl<rotors_comm::Octomap::Request,rotors_comm::Octomap::Response,rotors_comm_msg_OctomapRequest_common,rotors_comm_msg_OctomapResponse_common>>();
  }
  std::shared_ptr<MATLABSvcClientInterface> 
          rotors_comm_Octomap_service::generateSvcClientInterface(){
    return std::make_shared<ROSSvcClientImpl<rotors_comm::Octomap,rotors_comm::Octomap::Request,rotors_comm::Octomap::Response,rotors_comm_msg_OctomapRequest_common,rotors_comm_msg_OctomapResponse_common>>();
  }
#include "ROSbagTemplates.hpp" 
  std::shared_ptr<MATLABRosbagWriterInterface> 
          rotors_comm_Octomap_service::generateRosbagWriterInterface(ElementType type){
    std::shared_ptr<MATLABRosbagWriterInterface> ptr;
    if(type == eRequest){
        ptr = std::make_shared<ROSBagWriterImpl<rotors_comm::OctomapRequest,rotors_comm_msg_OctomapRequest_common>>();
    }else if(type == eResponse){
        ptr = std::make_shared<ROSBagWriterImpl<rotors_comm::OctomapResponse,rotors_comm_msg_OctomapResponse_common>>();
    }else{
        throw std::invalid_argument("Wrong input, Expected 'Request' or 'Response'");
    }
    return ptr;
  }
#include "register_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(rotors_comm_msg_OctomapRequest_common, MATLABROSMsgInterface<rotors_comm::OctomapRequest>)
CLASS_LOADER_REGISTER_CLASS(rotors_comm_msg_OctomapResponse_common, MATLABROSMsgInterface<rotors_comm::OctomapResponse>)
CLASS_LOADER_REGISTER_CLASS(rotors_comm_Octomap_service, ROSMsgElementInterfaceFactory)
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif //_MSC_VER
//gen-1

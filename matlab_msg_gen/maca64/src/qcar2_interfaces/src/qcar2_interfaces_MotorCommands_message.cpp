// Copyright 2020-2022 The MathWorks, Inc.
// Common copy functions for qcar2_interfaces/MotorCommands
#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable : 4100)
#pragma warning(disable : 4265)
#pragma warning(disable : 4456)
#pragma warning(disable : 4458)
#pragma warning(disable : 4946)
#pragma warning(disable : 4244)
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
#include "rclcpp/rclcpp.hpp"
#include "qcar2_interfaces/msg/motor_commands.hpp"
#include "visibility_control.h"
#include "class_loader/multi_library_class_loader.hpp"
#include "ROS2PubSubTemplates.hpp"
class QCAR2_INTERFACES_EXPORT ros2_qcar2_interfaces_msg_MotorCommands_common : public MATLABROS2MsgInterface<qcar2_interfaces::msg::MotorCommands> {
  public:
    virtual ~ros2_qcar2_interfaces_msg_MotorCommands_common(){}
    virtual void copy_from_struct(qcar2_interfaces::msg::MotorCommands* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const qcar2_interfaces::msg::MotorCommands* msg, MultiLibLoader loader, size_t size = 1);
};
  void ros2_qcar2_interfaces_msg_MotorCommands_common::copy_from_struct(qcar2_interfaces::msg::MotorCommands* msg, const matlab::data::Struct& arr,
               MultiLibLoader loader) {
    try {
        //motor_names
        const matlab::data::CellArray motor_names_cellarr = arr["motor_names"];
        size_t nelem = motor_names_cellarr.getNumberOfElements();
        for (size_t idx=0; idx < nelem; ++idx){
        	const matlab::data::CharArray motor_names_arr = motor_names_cellarr[idx];
        	msg->motor_names.push_back(motor_names_arr.toAscii());
        }
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'motor_names' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'motor_names' is wrong type; expected a string.");
    }
    try {
        //values
        const matlab::data::TypedArray<double> values_arr = arr["values"];
        size_t nelem = values_arr.getNumberOfElements();
        	msg->values.resize(nelem);
        	std::copy(values_arr.begin(), values_arr.begin()+nelem, msg->values.begin());
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'values' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'values' is wrong type; expected a double.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T ros2_qcar2_interfaces_msg_MotorCommands_common::get_arr(MDFactory_T& factory, const qcar2_interfaces::msg::MotorCommands* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType","motor_names","values"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("qcar2_interfaces/MotorCommands");
    // motor_names
    auto currentElement_motor_names = (msg + ctr)->motor_names;
    auto motor_namesoutCell = factory.createCellArray({currentElement_motor_names.size(),1});
    for(size_t idxin = 0; idxin < currentElement_motor_names.size(); ++ idxin){
    	motor_namesoutCell[idxin] = factory.createCharArray(currentElement_motor_names[idxin]);
    }
    outArray[ctr]["motor_names"] = motor_namesoutCell;
    // values
    auto currentElement_values = (msg + ctr)->values;
    outArray[ctr]["values"] = factory.createArray<qcar2_interfaces::msg::MotorCommands::_values_type::const_iterator, double>({currentElement_values.size(), 1}, currentElement_values.begin(), currentElement_values.end());
    }
    return std::move(outArray);
  } 
class QCAR2_INTERFACES_EXPORT ros2_qcar2_interfaces_MotorCommands_message : public ROS2MsgElementInterfaceFactory {
  public:
    virtual ~ros2_qcar2_interfaces_MotorCommands_message(){}
    virtual std::shared_ptr<MATLABPublisherInterface> generatePublisherInterface(ElementType /*type*/);
    virtual std::shared_ptr<MATLABSubscriberInterface> generateSubscriberInterface(ElementType /*type*/);
    virtual std::shared_ptr<void> generateCppMessage(ElementType /*type*/, const matlab::data::StructArray& /* arr */, MultiLibLoader /* loader */, std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* /*commonObjMap*/);
    virtual matlab::data::StructArray generateMLMessage(ElementType  /*type*/ ,void*  /* msg */, MultiLibLoader /* loader */ , std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* /*commonObjMap*/);
};  
  std::shared_ptr<MATLABPublisherInterface> 
          ros2_qcar2_interfaces_MotorCommands_message::generatePublisherInterface(ElementType /*type*/){
    return std::make_shared<ROS2PublisherImpl<qcar2_interfaces::msg::MotorCommands,ros2_qcar2_interfaces_msg_MotorCommands_common>>();
  }
  std::shared_ptr<MATLABSubscriberInterface> 
         ros2_qcar2_interfaces_MotorCommands_message::generateSubscriberInterface(ElementType /*type*/){
    return std::make_shared<ROS2SubscriberImpl<qcar2_interfaces::msg::MotorCommands,ros2_qcar2_interfaces_msg_MotorCommands_common>>();
  }
  std::shared_ptr<void> ros2_qcar2_interfaces_MotorCommands_message::generateCppMessage(ElementType /*type*/, 
                                           const matlab::data::StructArray& arr,
                                           MultiLibLoader loader,
                                           std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* commonObjMap){
    auto msg = std::make_shared<qcar2_interfaces::msg::MotorCommands>();
    ros2_qcar2_interfaces_msg_MotorCommands_common commonObj;
    commonObj.mCommonObjMap = commonObjMap;
    commonObj.copy_from_struct(msg.get(), arr[0], loader);
    return msg;
  }
  matlab::data::StructArray ros2_qcar2_interfaces_MotorCommands_message::generateMLMessage(ElementType  /*type*/ ,
                                                    void*  msg ,
                                                    MultiLibLoader  loader ,
                                                    std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>*  commonObjMap ){
    ros2_qcar2_interfaces_msg_MotorCommands_common commonObj;	
    commonObj.mCommonObjMap = commonObjMap;	
    MDFactory_T factory;
    return commonObj.get_arr(factory, (qcar2_interfaces::msg::MotorCommands*)msg, loader);			
 }
#include "class_loader/register_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(ros2_qcar2_interfaces_msg_MotorCommands_common, MATLABROS2MsgInterface<qcar2_interfaces::msg::MotorCommands>)
CLASS_LOADER_REGISTER_CLASS(ros2_qcar2_interfaces_MotorCommands_message, ROS2MsgElementInterfaceFactory)
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif //_MSC_VER
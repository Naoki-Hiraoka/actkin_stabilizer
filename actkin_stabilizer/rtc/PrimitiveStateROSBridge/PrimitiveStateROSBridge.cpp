#include "PrimitiveStateROSBridge.h"

PrimitiveStateROSBridge::PrimitiveStateROSBridge(RTC::Manager* manager):
  RTC::DataFlowComponentBase(manager),
  m_primitiveStateOut_("primitiveStateOut", m_primitiveStateROS_),
  m_primitiveStateIn_("primitiveStateIn", m_primitiveStateRTM_)
{
}

RTC::ReturnCode_t PrimitiveStateROSBridge::onInitialize(){
  addOutPort("primitiveStateOut", m_primitiveStateOut_);
  addInPort("primitiveStateIn", m_primitiveStateIn_);

  ros::NodeHandle pnh("~");

  primitiveStateSub_ = pnh.subscribe("input", 1, &PrimitiveStateROSBridge::onPrimitiveStateCB, this);
  primitiveStatePub_ = pnh.advertise<actkin_stabilizer::PrimitiveState>("output", 1);

  return RTC::RTC_OK;
}

RTC::ReturnCode_t PrimitiveStateROSBridge::onExecute(RTC::UniqueId ec_id){
  ros::spinOnce();
  if(this->m_primitiveStateIn_.isNew()){
    try {
      m_primitiveStateIn_.read();
      actkin_stabilizer::PrimitiveState msg;
      msg.header.stamp = ros::Time::now();
      // TODO

      primitiveStatePub_.publish(msg);
    }
    catch(const std::runtime_error &e)
      {
        ROS_ERROR_STREAM("[" << getInstanceName() << "] " << e.what());
      }
  }
  return RTC::RTC_OK;
}

void PrimitiveStateROSBridge::onPrimitiveStateCB(const actkin_stabilizer::PrimitiveState::ConstPtr& msg) {
  // TODO
  //m_landingHeight_.data.x = msg->x;
  m_primitiveStateOut_.write();
}

static const char* PrimitiveStateROSBridge_spec[] = {
  "implementation_id", "PrimitiveStateROSBridge",
  "type_name",         "PrimitiveStateROSBridge",
  "description",       "PrimitiveStateROSBridge component",
  "version",           "0.0",
  "vendor",            "Naoki-Hiraoka",
  "category",          "example",
  "activity_type",     "DataFlowComponent",
  "max_instance",      "10",
  "language",          "C++",
  "lang_type",         "compile",
  ""
};

extern "C"{
    void PrimitiveStateROSBridgeInit(RTC::Manager* manager) {
        RTC::Properties profile(PrimitiveStateROSBridge_spec);
        manager->registerFactory(profile, RTC::Create<PrimitiveStateROSBridge>, RTC::Delete<PrimitiveStateROSBridge>);
    }
};

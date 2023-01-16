#ifndef PrimitiveStateROSBridge_H
#define PrimitiveStateROSBridge_H

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/DataOutPort.h>
#include <rtm/DataInPort.h>

#include <actkin_stabilizer/idl/ActKinStabilizerService.hh>
#include <actkin_stabilizer/PrimitiveState.h>

#include <ros/ros.h>

class PrimitiveStateROSBridge : public RTC::DataFlowComponentBase{
protected:
  ros::NodeHandle nh;

  actkin_stabilizer::PrimitiveStateIdl m_primitiveStateRTM_;
  RTC::InPort <actkin_stabilizer::PrimitiveStateIdl> m_primitiveStateIn_;
  ros::Publisher primitiveStatePub_;

  ros::Subscriber primitiveStateSub_;
  actkin_stabilizer::PrimitiveStateIdl m_primitiveStateROS_;
  RTC::OutPort <actkin_stabilizer::PrimitiveStateIdl> m_primitiveStateOut_;

public:
  PrimitiveStateROSBridge(RTC::Manager* manager);
  virtual RTC::ReturnCode_t onInitialize();
  virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

  void onPrimitiveStateCB(const actkin_stabilizer::PrimitiveState::ConstPtr& msg);

};

extern "C"
{
  void PrimitiveStateROSBridgeInit(RTC::Manager* manager);
};

#endif // PrimitiveStateROSBridge_H

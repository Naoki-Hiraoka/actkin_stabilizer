#ifndef ActKinStabilizer_H
#define ActKinStabilizer_H

#include <time.h>
#include <mutex>
#include <unordered_set>

#include <rtm/idl/BasicDataType.hh>
#include <rtm/idl/ExtendedDataTypes.hh>
#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>
#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/ExtendedDataTypesSkel.h>
#include <rtm/CorbaNaming.h>

#include <cnoid/Body>

#include <cpp_filters/TwoPointInterpolator.h>

#include <collision_checker_msgs/idl/Collision.hh>
#include <actkin_stabilizer_msgs/idl/ActKinStabilizer.hh>
#include <contact_state_msgs/idl/ContactState.hh>

#include "ActKinStabilizerService_impl.h"
#include "State.h"
#include "Goal.h"
#include "ResolvedAccelerationController.h"

class ActKinStabilizer : public RTC::DataFlowComponentBase{
public:
  ActKinStabilizer(RTC::Manager* manager);
  virtual RTC::ReturnCode_t onInitialize();
  virtual RTC::ReturnCode_t onFinalize();
  virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);
  virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);
  virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

  bool startStabilizer();
  bool stopStabilizer();
  bool setActKinStabilizerParam(const actkin_stabilizer::ActKinStabilizerService::ActKinStabilizerParam& i_param);
  bool getActKinStabilizerParam(actkin_stabilizer::ActKinStabilizerService::ActKinStabilizerParam& i_param);

  bool setRefState(const actkin_stabilizer_msgs::RefStateIdl& i_param);

protected:
  std::mutex mutex_;

  unsigned long long loop_;

  class Ports {
  public:
    Ports();
    void onInitialize(ActKinStabilizer* component);

    RTC::TimedDoubleSeq m_qAct_;
    RTC::InPort<RTC::TimedDoubleSeq> m_qActIn_;
    RTC::TimedDoubleSeq m_dqAct_;
    RTC::InPort<RTC::TimedDoubleSeq> m_dqActIn_;
    RTC::TimedPose3D m_actBasePose_;
    RTC::InPort<RTC::TimedPose3D> m_actBasePoseIn_;
    RTC::TimedVelocity3D m_actBaseVel_;
    RTC::InPort<RTC::TimedVelocity3D> m_actBaseVelIn_;
    contact_state_msgs::TimedContactSeq m_actContactState_;
    RTC::InPort<contact_state_msgs::TimedContactSeq> m_actContactStateIn_;

    actkin_stabilizer_msgs::RefStateIdl m_refState_;
    RTC::InPort<actkin_stabilizer_msgs::RefStateIdl> m_refStateIn_;
    bool m_refStateUpdatedByService_ = false;

    collision_checker_msgs::TimedCollisionSeq m_selfCollision_; // generate frame. genRobotの自己干渉の最近傍点
    RTC::InPort<collision_checker_msgs::TimedCollisionSeq> m_selfCollisionIn_;

    RTC::TimedDoubleSeq m_tau_;
    RTC::OutPort<RTC::TimedDoubleSeq> m_tauOut_;

    ActKinStabilizerService_impl m_service0_;
    RTC::CorbaPort m_ActKinStabilizerServicePort_;
  };
  Ports ports_;

  class ControlMode{
  public:
    /*
      MODE_IDLE -> startStabilizer() -> MODE_ST
      MODE_ST -> stopStabilizer() -> MODE_IDLE

      MODE_IDLEの場合はstateのみ受け取り他は何もしない.
      MODE_STの場合はgoalも受け取り、torqueを出力する.
      切り替えの連続性を担保するにはこのRTCではない.
     */
    enum Mode_enum{ MODE_IDLE, MODE_ST};
    enum Transition_enum{ START_ST, STOP_ST};
  private:
    Mode_enum current, previous, next;
  public:
    ControlMode(){ reset(); }
    void reset(){ current = previous = next = MODE_IDLE; }
    bool setNextTransition(const Transition_enum request){
      switch(request){
      case START_ST:
        if(current == MODE_IDLE){ next = MODE_ST; return true; }else{ return false; }
      case STOP_ST:
        if(current == MODE_ST){ next = MODE_IDLE; return true; }else{ return false; }
      default:
        return false;
      }
    }
    void update(){
      previous = current; current = next;
    }
    Mode_enum now() const{ return current; }
    Mode_enum pre() const{ return previous; }
    bool isSyncToSTInit() const{ return (current != previous) && (current==MODE_ST);}
    bool isSyncToIdleInit() const{ return (current != previous) && (current==MODE_IDLE);}
    bool isSTRunning() const{ return (current==MODE_ST) ;}
  };
  ControlMode mode_;

  actkin_stabilizer::State state_;
  actkin_stabilizer::Goal goal_;
  actkin_stabilizer::ResolvedAccelerationController resolvedAccelerationController_;

protected:
  // utility functions
  bool getProperty(const std::string& key, std::string& ret);

  static bool readInPortDataForState(ActKinStabilizer::Ports& ports, const std::string& instance_name, const double& dt,
                                     actkin_stabilizer::State& state);
  static bool readInPortDataForGoal(ActKinStabilizer::Ports& ports, const std::string& instance_name, const double& dt, const actkin_stabilizer::State& state,
                                    actkin_stabilizer::Goal& goal);
  static bool writeOutPortData(const actkin_stabilizer::State& state, const ActKinStabilizer::ControlMode& mode,
                               ActKinStabilizer::Ports& ports);

};


extern "C"
{
  void ActKinStabilizerInit(RTC::Manager* manager);
};

#endif // ActKinStabilizer_H

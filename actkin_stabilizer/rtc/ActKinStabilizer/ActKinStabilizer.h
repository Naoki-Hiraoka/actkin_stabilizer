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
#include "ResolvedAccelerationController.h"
#include "WrenchDistributor.h"

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

protected:
  std::mutex mutex_;

  unsigned long long loop_;

  class Ports {
  public:
    Ports();
    void onInitialize(ActKinStabilizer* component);

    actkin_stabilizer_msgs::RefStateIdl m_refState_;
    RTC::InPort<actkin_stabilizer_msgs::RefStateIdl> m_refStateIn_;

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

      MODE_IDLEの場合は何もしない.
      MODE_STの場合は与えられた目標を解釈し、
      切り替えの連続性を担保するにはこのRTCではない.
      MODE_SYNC_TO*の時間はtransition_timeの時間をかけて遷移するが、少なくとも1周期はMODE_SYNC_TO*を経由する.
      MODE_SYNC_TO*では、基本的に次のMODEと同じ処理が行われるが、出力時に前回のMODEの出力から補間するような軌道に加工されることで出力の連続性を確保する
      補間している途中で別のmodeに切り替わることは無いので、そこは安心してプログラムを書いてよい(例外はonActivated). 同様に、remainTimeが突然減ったり増えたりすることもない

      MODE_ABC: contactに基づきhrpsys_odom座標系を更新する
      MODE_ST: トルク制御指令値を求める
     */
    enum Mode_enum{ MODE_IDLE, MODE_SYNC_TO_ABC, MODE_ABC, MODE_SYNC_TO_ST, MODE_ST, MODE_SYNC_TO_STOPST, MODE_SYNC_TO_IDLE};
    enum Transition_enum{ START_ABC, STOP_ABC, START_ST, STOP_ST};
    double abc_start_transition_time, abc_stop_transition_time, st_start_transition_time, st_stop_transition_time;
  private:
    Mode_enum current, previous, next;
    cpp_filters::TwoPointInterpolator<double> transitionInterpolator = cpp_filters::TwoPointInterpolator<double>(1.0, 0.0, 0.0, cpp_filters::LINEAR); // SYNC_TO状態のとき、0 -> 1. SYNC_TO以外のとき、常に1
  public:
    ControlMode(){ reset(); abc_start_transition_time = 0.5; abc_stop_transition_time = 2.0; st_start_transition_time = 0.5; st_stop_transition_time = 0.5;}
    void reset(){ current = previous = next = MODE_IDLE; transitionInterpolator.reset(1.0);}
    bool setNextTransition(const Transition_enum request){
      switch(request){
      case START_ABC:
        if(current == MODE_IDLE){ next = MODE_SYNC_TO_ABC; return true; }else{ return false; }
      case STOP_ABC:
        if(current == MODE_ABC){ next = MODE_SYNC_TO_IDLE; return true; }else{ return false; }
      case START_ST:
        if(current == MODE_ABC){ next = MODE_SYNC_TO_ST; return true; }else{ return false; }
      case STOP_ST:
        if(current == MODE_ST){ next = MODE_SYNC_TO_STOPST; return true; }else{ return false; }
      default:
        return false;
      }
    }
    void update(double dt){
      if(current != next) {
        previous = current; current = next;
        switch(current){
        case MODE_SYNC_TO_ABC:
          transitionInterpolator.reset(0.0);
          transitionInterpolator.setGoal(1.0, abc_start_transition_time); break;
        case MODE_SYNC_TO_IDLE:
          transitionInterpolator.reset(0.0);
          transitionInterpolator.setGoal(1.0, abc_stop_transition_time); break;
        case MODE_SYNC_TO_ST:
          transitionInterpolator.reset(0.0);
          transitionInterpolator.setGoal(1.0, st_start_transition_time); break;
        case MODE_SYNC_TO_STOPST:
          transitionInterpolator.reset(0.0);
          transitionInterpolator.setGoal(1.0, st_stop_transition_time); break;
        default:
          break;
        }
      }else{
        previous = current;
        transitionInterpolator.interpolate(dt);
        if(transitionInterpolator.isEmpty()){
          switch(current){
          case MODE_SYNC_TO_ABC:
            current = next = MODE_ABC; break;
          case MODE_SYNC_TO_IDLE:
            current = next = MODE_IDLE; break;
          case MODE_SYNC_TO_ST:
            current = next = MODE_ST; break;
          case MODE_SYNC_TO_STOPST:
            current = next = MODE_ABC; break;
          default:
            break;
          }
        }
      }
    }
    double remainTime() const{ return transitionInterpolator.remain_time();}
    double transitionRatio() const{ return transitionInterpolator.value();}
    Mode_enum now() const{ return current; }
    Mode_enum pre() const{ return previous; }
    bool isABCRunning() const{ return (current==MODE_SYNC_TO_ABC) || (current==MODE_ABC) || (current==MODE_SYNC_TO_ST) || (current==MODE_ST) || (current==MODE_SYNC_TO_STOPST) ;}
    bool isSyncToABCInit() const{ return (current != previous) && (current==MODE_SYNC_TO_ABC);}
    bool isSyncToIdleInit() const{ return (current != previous) && (current==MODE_SYNC_TO_IDLE);}
    bool isSyncToSTInit() const{ return (current != previous) && (current==MODE_SYNC_TO_ST);}
    bool isSyncToStopSTInit() const{ return (current != previous) && (current==MODE_SYNC_TO_STOPST);}
    bool isSTRunning() const{ return (current==MODE_SYNC_TO_ST) || (current==MODE_ST) ;}
  };
  ControlMode mode_;
  cpp_filters::TwoPointInterpolatorSE3 outputRootPoseFilter_{cnoid::Position::Identity(),cnoid::Vector6::Zero(),cnoid::Vector6::Zero(),cpp_filters::HOFFARBIB};

  State gaitParam_;
  ResolvedAccelerationController resolvedAccelerationController_;
  WrenchDistributor wrenchDistributor_;

protected:
  // utility functions
  bool getProperty(const std::string& key, std::string& ret);

  static bool readInPortData(const double& dt, const State& gaitParam, const ActKinStabilizer::ControlMode& mode, ActKinStabilizer::Ports& ports, cnoid::BodyPtr refRobotRaw, cnoid::BodyPtr actRobotRaw, std::vector<State::Collision>& selfCollision, std::unordered_map<std::string, std::shared_ptr<Contact> >& contacts, std::unordered_map<std::string, std::shared_ptr<Attention> >& attentions, std::vector<std::shared_ptr<Object> >& activeObjects, std::vector<std::shared_ptr<Contact> >& activeContacts, std::vector<std::vector<std::shared_ptr<Attention> > >& prioritizedAttentions);
  static bool writeOutPortData(ActKinStabilizer::Ports& ports, const ActKinStabilizer::ControlMode& mode, double dt, const State& gaitParam, cpp_filters::TwoPointInterpolatorSE3& outputRootPoseFilter);

};


extern "C"
{
  void ActKinStabilizerInit(RTC::Manager* manager);
};

#endif // ActKinStabilizer_H

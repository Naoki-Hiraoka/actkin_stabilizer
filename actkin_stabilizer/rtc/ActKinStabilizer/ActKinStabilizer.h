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
#include <auto_stabilizer_msgs/idl/AutoStabilizer.hh>

#include "ActKinStabilizerService_impl.h"
#include "GaitParam.h"
#include "ActToGenFrameConverter.h"
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

  bool startAutoBalancer();
  bool stopAutoBalancer();
  bool startStabilizer();
  bool stopStabilizer();
  bool setPrimitiveState(const actkin_stabilizer::PrimitiveStateIdl& command);
  bool getPrimitiveState(actkin_stabilizer::PrimitiveStateIdl& command);
  bool resetPrimitiveState(const actkin_stabilizer::PrimitiveStateIdl& command);
  bool goActual();
  bool loadObject(const std::string& name, const std::string& file);
  bool unloadObject(const std::string& name);
  bool setObjectState(const actkin_stabilizer::ObjectStateIdl& obj);
  bool setObjectStates(const actkin_stabilizer::ObjectStateIdlSeq& objs);
  bool getObjectStates(actkin_stabilizer::ObjectStateIdlSeq& objs);
  bool setActKinStabilizerParam(const actkin_stabilizer::ActKinStabilizerService::ActKinStabilizerParam& i_param);
  bool getActKinStabilizerParam(actkin_stabilizer::ActKinStabilizerService::ActKinStabilizerParam& i_param);

protected:
  std::mutex mutex_;

  unsigned int debugLevel_;
  unsigned long long loop_;

  class Ports {
  public:
    Ports();

    RTC::TimedDoubleSeq m_qRef_;
    RTC::InPort<RTC::TimedDoubleSeq> m_qRefIn_;
    RTC::TimedDoubleSeq m_refTau_;
    RTC::InPort<RTC::TimedDoubleSeq> m_refTauIn_;
    RTC::TimedPoint3D m_refBasePos_; // generate frame
    RTC::InPort<RTC::TimedPoint3D> m_refBasePosIn_;
    RTC::TimedDoubleSeq m_qAct_;
    RTC::InPort<RTC::TimedDoubleSeq> m_qActIn_;
    RTC::TimedDoubleSeq m_dqAct_;
    RTC::InPort<RTC::TimedDoubleSeq> m_dqActIn_;
    RTC::TimedOrientation3D m_actImu_; // Actual Imu World Frame. robot???gyrometer???????????????RateGyroSensor??????????????????
    RTC::InPort<RTC::TimedOrientation3D> m_actImuIn_;
    collision_checker_msgs::TimedCollisionSeq m_selfCollision_; // generate frame. genRobot??????????????????????????????
    RTC::InPort<collision_checker_msgs::TimedCollisionSeq> m_selfCollisionIn_;
    actkin_stabilizer::PrimitiveStateIdl m_primitiveCommand_;
    RTC::InPort<actkin_stabilizer::PrimitiveStateIdl> m_primitiveCommandIn_;

    RTC::TimedDoubleSeq m_genTau_;
    RTC::OutPort<RTC::TimedDoubleSeq> m_genTauOut_;
    RTC::TimedPose3D m_actBasePose_; // Generate World frame
    RTC::OutPort<RTC::TimedPose3D> m_actBasePoseOut_;
    RTC::TimedDoubleSeq m_actBaseTform_;  // Generate World frame
    RTC::OutPort<RTC::TimedDoubleSeq> m_actBaseTformOut_; // for HrpsysSeqStateROSBridge
    RTC::TimedPoint3D m_actBasePos_; // Generate World frame
    RTC::OutPort<RTC::TimedPoint3D> m_actBasePosOut_; // for old RTCs
    RTC::TimedOrientation3D m_actBaseRpy_; // Generate World frame
    RTC::OutPort<RTC::TimedOrientation3D> m_actBaseRpyOut_; // for old RTCs
    actkin_stabilizer::ObjectStateIdlSeq m_objectStates_; // Generate World frame
    RTC::OutPort<actkin_stabilizer::ObjectStateIdlSeq> m_objectStatesOut_;
    actkin_stabilizer::PrimitiveStateIdl m_primitiveState_;
    RTC::OutPort<actkin_stabilizer::PrimitiveStateIdl> m_primitiveStateOut_;

    ActKinStabilizerService_impl m_service0_;
    RTC::CorbaPort m_ActKinStabilizerServicePort_;

    // only for log
    RTC::TimedPoint3D m_actCog_; // Generate World frame
    RTC::OutPort<RTC::TimedPoint3D> m_actCogOut_; // for log
  };
  Ports ports_;

  class ControlMode{
  public:
    /*
      MODE_IDLE -> startAutoBalancer() -> MODE_SYNC_TO_ABC -> MODE_ABC
      MODE_ABC -> stopAutoBalancer() -> MODE_SYNC_TO_IDLE -> MODE_IDLE

      MODE_ABC -> startStabilizer() -> MODE_SYNC_TO_ST -> MODE_ST
      MODE_ST -> stopStabilizer() -> MODE_SYNC_TO_STOPST -> MODE_ABC

      MODE_SYNC_TO*????????????transition_time??????????????????????????????????????????????????????1?????????MODE_SYNC_TO*???????????????.
      MODE_SYNC_TO*???????????????????????????MODE?????????????????????????????????????????????????????????MODE??????????????????????????????????????????????????????????????????????????????????????????????????????
      ?????????????????????????????????mode????????????????????????????????????????????????????????????????????????????????????????????????(?????????onActivated). ????????????remainTime??????????????????????????????????????????????????????

      MODE_ABC: contact????????????hrpsys_odom????????????????????????
      MODE_ST: ????????????????????????????????????
     */
    enum Mode_enum{ MODE_IDLE, MODE_SYNC_TO_ABC, MODE_ABC, MODE_SYNC_TO_ST, MODE_ST, MODE_SYNC_TO_STOPST, MODE_SYNC_TO_IDLE};
    enum Transition_enum{ START_ABC, STOP_ABC, START_ST, STOP_ST};
    double abc_start_transition_time, abc_stop_transition_time, st_start_transition_time, st_stop_transition_time;
  private:
    Mode_enum current, previous, next;
    cpp_filters::TwoPointInterpolator<double> transitionInterpolator = cpp_filters::TwoPointInterpolator<double>(1.0, 0.0, 0.0, cpp_filters::LINEAR); // SYNC_TO??????????????????0 -> 1. SYNC_TO????????????????????????1
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

  GaitParam gaitParam_;
  ActToGenFrameConverter actToGenFrameConverter_;
  ResolvedAccelerationController resolvedAccelerationController_;
  WrenchDistributor wrenchDistributor_;

protected:
  // utility functions
  bool getProperty(const std::string& key, std::string& ret);

  static bool readInPortData(const double& dt, const GaitParam& gaitParam, const ActKinStabilizer::ControlMode& mode, ActKinStabilizer::Ports& ports, cnoid::BodyPtr refRobotRaw, cnoid::BodyPtr actRobotRaw, std::vector<GaitParam::Collision>& selfCollision, std::unordered_map<std::string, std::shared_ptr<Contact> >& contacts, std::unordered_map<std::string, std::shared_ptr<Attention> >& attentions, std::vector<std::shared_ptr<Object> >& activeObjects, std::vector<std::shared_ptr<Contact> >& activeContacts, std::vector<std::vector<std::shared_ptr<Attention> > >& prioritizedAttentions);
  static bool writeOutPortData(ActKinStabilizer::Ports& ports, const ActKinStabilizer::ControlMode& mode, double dt, const GaitParam& gaitParam, cpp_filters::TwoPointInterpolatorSE3& outputRootPoseFilter);

};


extern "C"
{
  void ActKinStabilizerInit(RTC::Manager* manager);
};

#endif // ActKinStabilizer_H

#ifndef ActKinStabilizer_H
#define ActKinStabilizer_H

#include <memory>
#include <map>
#include <time.h>
#include <mutex>

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

// #include <cpp_filters/IIRFilter.h>
// #include <joint_limit_table/JointLimitTable.h>

#include <hrpsys/idl/RobotHardwareService.hh>
#include <collision_checker_msgs/idl/Collision.hh>
#include <auto_stabilizer_msgs/idl/AutoStabilizer.hh>

#include "ActKinStabilizerService_impl.h"
#include "GaitParam.h"
#include "RefToGenFrameConverter.h"
#include "ActToGenFrameConverter.h"
#include "LegManualController.h"
#include "FootStepGenerator.h"
#include "LegCoordsGenerator.h"
#include "ImpedanceController.h"
#include "Stabilizer.h"
#include "ExternalForceHandler.h"
#include "CmdVelGenerator.h"

class ActKinStabilizer : public RTC::DataFlowComponentBase{
public:
  ActKinStabilizer(RTC::Manager* manager);
  virtual RTC::ReturnCode_t onInitialize();
  virtual RTC::ReturnCode_t onFinalize();
  virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);
  virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);
  virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

  bool setFootSteps(const actkin_stabilizer::ActKinStabilizerService::FootstepSequence& fs);
  bool startAutoBalancer();
  bool stopAutoBalancer();
  bool setActKinStabilizerParam(const actkin_stabilizer::ActKinStabilizerService::ActKinStabilizerParam& i_param);
  bool getActKinStabilizerParam(actkin_stabilizer::ActKinStabilizerService::ActKinStabilizerParam& i_param);
  bool startStabilizer(void);
  bool stopStabilizer(void);

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
    RTC::TimedDoubleSeq m_qAct_;
    RTC::InPort<RTC::TimedDoubleSeq> m_qActIn_;
    RTC::TimedDoubleSeq m_dqAct_;
    RTC::InPort<RTC::TimedDoubleSeq> m_dqActIn_;
    RTC::TimedOrientation3D m_actImu_; // Actual Imu World Frame. robotのgyrometerという名のRateGyroSensorの傾きを表す
    RTC::InPort<RTC::TimedOrientation3D> m_actImuIn_;
    collision_checker_msgs::TimedCollisionSeq m_selfCollision_; // generate frame. genRobotの自己干渉の最近傍点
    RTC::InPort<collision_checker_msgs::TimedCollisionSeq> m_selfCollisionIn_;

    RTC::TimedDoubleSeq m_genTau_;
    RTC::OutPort<RTC::TimedDoubleSeq> m_genTauOut_;
    RTC::TimedPose3D m_actBasePose_; // Generate World frame
    RTC::OutPort<RTC::TimedPose3D> m_actBasePoseOut_;
    RTC::TimedDoubleSeq m_actBaseTform_;  // Generate World frame
    RTC::OutPort<RTC::TimedDoubleSeq> m_actBaseTformOut_; // for HrpsysSeqStateROSBridge

    ActKinStabilizerService_impl m_service0_;
    RTC::CorbaPort m_ActKinStabilizerServicePort_;

    // only for log
    RTC::TimedPoint3D m_actBasePos_; // Generate World frame
    RTC::OutPort<RTC::TimedPoint3D> m_actBasePosOut_; // for log
    RTC::TimedOrientation3D m_actBaseRpy_; // Generate World frame
    RTC::OutPort<RTC::TimedOrientation3D> m_actBaseRpyOut_; // for log
    RTC::TimedPoint3D m_actCog_; // Generate World frame
    RTC::OutPort<RTC::TimedPoint3D> m_actCogOut_; // for log
  };
  Ports ports_;

  class ControlMode{
  public:
    /*
      MODE_IDLE -> startAutoBalancer() -> MODE_SYNC_TO_ABC(odomの初期化) -> MODE_ABC
      MODE_ABC -> stopAutoBalancer() -> MODE_SYNC_TO_IDLE -> MODE_IDLE

      MODE_ABC or MODE_EMG (各タスクは常にactualで上書きされる) -> startStabilizer() -> MODE_SYNC_TO_ST -> MODE_ST
      MODE_ST(指令関節角度は常にactualで上書きされる) -> stopStabilizer() -> MODE_SYNC_TO_EMG -> MODE_EMG (指令関節角度は常に動かない)

      MODE_EMG (指令関節角度は常に動かない) -> releaseEmergencyStop() -> MODE_SYMC_TO_RELEASE_EMG -> MODE_ABC

      MODE_SYNC_TO*の時間はtransition_timeの時間をかけて遷移するが、少なくとも1周期はMODE_SYNC_TO*を経由する.
      MODE_SYNC_TO*では、基本的に次のMODEと同じ処理が行われるが、出力時に前回のMODEの出力から補間するような軌道に加工されることで出力の連続性を確保する
      補間している途中で別のmodeに切り替わることは無いので、そこは安心してプログラムを書いてよい(例外はonActivated). 同様に、remainTimeが突然減ったり増えたりすることもない

      MODE_ABC: 位置制御. 出力する指令関節角度は変化しない
      MODE_ST: トルク制御. 出力する指令関節角度はactualの値で更新する.
     */
    enum Mode_enum{ MODE_IDLE, MODE_SYNC_TO_ST, MODE_ST, MODE_SYNC_TO_IDLE};
    enum Transition_enum{ START_ST, STOP_ST};
    double st_start_transition_time, st_stop_transition_time;
  private:
    Mode_enum current, previous, next;
    cpp_filters::TwoPointInterpolator<double> transitionInterpolator = cpp_filters::TwoPointInterpolator<double>(1.0, 0.0, 0.0, cpp_filters::LINEAR); // 0 -> 1
  public:
    ControlMode(){ reset(); abc_start_transition_time = 0.5; abc_stop_transition_time = 2.0; st_start_transition_time = 0.5; st_stop_transition_time = 0.5; release_emergency_transition_time = 2.0;}
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
        if(current == MODE_ST){ next = MODE_SYNC_TO_EMG; return true; }else{ return false; }
      case RELEASE_EMG:
        if(current == MODE_EMG){ next = MODE_SYNC_TO_RELEASE_EMG; return true; }else{ return false; }
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
        case MODE_SYNC_TO_EMG:
          transitionInterpolator.reset(0.0);
          transitionInterpolator.setGoal(1.0, st_stop_transition_time); break;
        case MODE_SYNC_TO_RELEASE_EMG:
          transitionInterpolator.reset(0.0);
          transitionInterpolator.setGoal(1.0, release_emergency_transition_time); break;
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
          case MODE_SYNC_TO_EMG:
            current = next = MODE_EMG; break;
          case MODE_SYNC_TO_RELEASE_EMG:
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
    bool isABCRunning() const{ return (current==MODE_SYNC_TO_ABC) || (current==MODE_ABC) || (current==MODE_SYNC_TO_ST) || (current==MODE_ST) || (current==MODE_SYNC_TO_EMG) || (current==MODE_EMG) || (current==MODE_SYNC_TO_RELEASE_EMG) ;}
    bool isSyncToABC() const{ return current==MODE_SYNC_TO_ABC;}
    bool isSyncToABCInit() const{ return (current != previous) && isSyncToABC();}
    bool isSyncToIdle() const{ return current==MODE_SYNC_TO_IDLE;}
    bool isSyncToIdleInit() const{ return (current != previous) && isSyncToIdle();}
    bool isSyncToST() const{ return current == MODE_SYNC_TO_ST;}
    bool isSyncToSTInit() const{ return (current != previous) && isSyncToST();}
    bool isSyncToStopST() const{ return current == MODE_SYNC_TO_EMG;}
    bool isSyncToStopSTInit() const{ return (current != previous) && isSyncToStopST();}
    bool isSTRunning() const{ return (current==MODE_SYNC_TO_ST) || (current==MODE_ST) ;}
  };
  ControlMode mode_;

  GaitParam gaitParam_;

  RefToGenFrameConverter refToGenFrameConverter_;
  ActToGenFrameConverter actToGenFrameConverter_;
  ExternalForceHandler externalForceHandler_;
  ImpedanceController impedanceController_;
  LegManualController legManualController_;
  CmdVelGenerator cmdVelGenerator_;
  FootStepGenerator footStepGenerator_;
  LegCoordsGenerator legCoordsGenerator_;
  Stabilizer stabilizer_;

protected:
  // utility functions
  bool getProperty(const std::string& key, std::string& ret);

  static bool readInPortData(const double& dt, const GaitParam& gaitParam, const ActKinStabilizer::ControlMode& mode, ActKinStabilizer::Ports& ports, cnoid::BodyPtr refRobotRaw, cnoid::BodyPtr actRobotRaw, std::vector<cnoid::Vector6>& refEEWrenchOrigin, std::vector<cpp_filters::TwoPointInterpolatorSE3>& refEEPoseRaw, std::vector<GaitParam::Collision>& selfCollision, std::vector<std::vector<cnoid::Vector3> >& steppableRegion, std::vector<double>& steppableHeight, double& relLandingHeight, cnoid::Vector3& relLandingNormal);
  static bool execActKinStabilizer(const ActKinStabilizer::ControlMode& mode, GaitParam& gaitParam, double dt, const FootStepGenerator& footStepGenerator, const LegCoordsGenerator& legCoordsGenerator, const RefToGenFrameConverter& refToGenFrameConverter, const ActToGenFrameConverter& actToGenFrameConverter, const ImpedanceController& impedanceController, const Stabilizer& stabilizer, const ExternalForceHandler& externalForceHandler, const LegManualController& legManualController, const CmdVelGenerator& cmdVelGenerator);
  static bool writeOutPortData(ActKinStabilizer::Ports& ports, const ActKinStabilizer::ControlMode& mode, double dt, const GaitParam& gaitParam, cpp_filters::TwoPointInterpolatorSE3& outputRootPoseFilter, std::vector<cpp_filters::TwoPointInterpolator<double> >& outputJointAngleFilter);
};


extern "C"
{
  void ActKinStabilizerInit(RTC::Manager* manager);
};

#endif // ActKinStabilizer_H

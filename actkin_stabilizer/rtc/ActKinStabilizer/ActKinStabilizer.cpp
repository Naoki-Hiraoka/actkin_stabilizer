#include "ActKinStabilizer.h"
#include <cnoid/BodyLoader>
#include <cnoid/ForceSensor>
#include <cnoid/RateGyroSensor>
#include <cnoid/ValueTree>
#include <cnoid/EigenUtil>
#include "MathUtil.h"
#include "CnoidBodyUtil.h"
#include <limits>
#include <eigen_rtm_conversions/eigen_rtm_conversions.h>

static const char* ActKinStabilizer_spec[] = {
  "implementation_id", "ActKinStabilizer",
  "type_name",         "ActKinStabilizer",
  "description",       "ActKinStabilizer component",
  "version",           "0.0",
  "vendor",            "Naoki-Hiraoka",
  "category",          "example",
  "activity_type",     "DataFlowComponent",
  "max_instance",      "10",
  "language",          "C++",
  "lang_type",         "compile",
  ""
};

ActKinStabilizer::Ports::Ports() :
  m_qActIn_("qAct", m_qAct_),
  m_dqActIn_("dqAct", m_dqAct_),
  m_actBasePoseIn_("actBasePoseIn", m_actBasePose_),
  m_actBaseVelIn_("actBaseVelIn", m_actBaseVel_),
  m_actContactStateIn_("actContactStateIn", m_actContactState_),
  m_refStateIn_("refStateIn", m_refState_),
  m_selfCollisionIn_("selfCollisionIn", m_selfCollision_),

  m_tauOut_("tauOut", m_tau_),

  m_ActKinStabilizerServicePort_("ActKinStabilizerService") {

  m_actBasePose_.data.position.x = 0.0;
  m_actBasePose_.data.position.y = 0.0;
  m_actBasePose_.data.position.z = 0.0;
  m_actBasePose_.data.orientation.r = 0.0;
  m_actBasePose_.data.orientation.p = 0.0;
  m_actBasePose_.data.orientation.y = 0.0;
  m_actBaseVel_.data.vx = 0.0;
  m_actBaseVel_.data.vy = 0.0;
  m_actBaseVel_.data.vz = 0.0;
  m_actBaseVel_.data.vr = 0.0;
  m_actBaseVel_.data.vp = 0.0;
  m_actBaseVel_.data.va = 0.0;
}

void ActKinStabilizer::Ports::onInitialize(ActKinStabilizer* component) {
  component->addInPort("qAct", this->m_qActIn_);
  component->addInPort("dqAct", this->m_dqActIn_);
  component->addInPort("actBasePoseIn", this->m_actBasePoseIn_);
  component->addInPort("actBaseVelIn", this->m_actBaseVelIn_);
  component->addInPort("actContactStateIn", this->m_actContactStateIn_);
  component->addInPort("refStateIn", this->m_refStateIn_);
  component->addInPort("selfCollisionIn", this->m_selfCollisionIn_);
  component->addOutPort("tauOut", this->m_tauOut_);

  this->m_ActKinStabilizerServicePort_.registerProvider("service0", "ActKinStabilizerService", this->m_service0_);
  component->addPort(this->m_ActKinStabilizerServicePort_);
  return;
}

ActKinStabilizer::ActKinStabilizer(RTC::Manager* manager) : RTC::DataFlowComponentBase(manager),
  ports_()
{
  this->ports_.m_service0_.setComp(this);
}

RTC::ReturnCode_t ActKinStabilizer::onInitialize(){
  std::cerr << "[" << m_profile.instance_name << "] onInitialize()" << std::endl;
  this->ports_.onInitialize(this);

  {
    // load robot model
    cnoid::BodyPtr robot;
    {
      cnoid::BodyLoader bodyLoader;
      std::string fileName; this->getProperty("model", fileName);
      if (fileName.find("file://") == 0) fileName.erase(0, strlen("file://"));
      robot = bodyLoader.load(fileName);
      if(!robot){
        std::cerr << "\x1b[31m[" << this->m_profile.instance_name << "] " << "failed to load model[" << fileName << "]" << "\x1b[39m" << std::endl;
        return RTC::RTC_ERROR;
      }
      if(!robot->rootLink()->isFreeJoint()){
        std::cerr << "\x1b[31m[" << this->m_profile.instance_name << "] " << "rootLink is not FreeJoint [" << fileName << "]" << "\x1b[39m" << std::endl;
        return RTC::RTC_ERROR;
      }

      // apply margin to jointlimit
      for(int i=0;i<robot->numJoints();i++){
        cnoid::LinkPtr joint = robot->joint(i);
        if(joint->q_upper() - joint->q_lower() > 0.002){
          joint->setJointRange(joint->q_lower()+0.001,joint->q_upper()-0.001);
        }
        // JointVelocityについて. 1.0だと安全.4.0は脚.10.0はlapid manipulation らしい. limitを小さくしすぎた状態で、速い指令を送ると、狭いlimitの中で高優先度タスクを頑張って満たそうとすることで、低優先度タスクを満たす余裕がなくエラーが大きくなってしまうことに注意.
        if(joint->dq_upper() - joint->dq_lower() > 0.02){
          joint->setJointVelocityRange(joint->dq_lower()+0.01,joint->dq_upper()-0.01);
        }
      }
    }

    // joint limit table
    std::vector<std::vector<std::shared_ptr<joint_limit_table::JointLimitTable> > > jointLimitTables(robot->numJoints());
    {
      std::string jointLimitTableStr; this->getProperty("joint_limit_table",jointLimitTableStr);
      std::vector<std::shared_ptr<joint_limit_table::JointLimitTable> > jointLimitTablesRaw = joint_limit_table::readJointLimitTablesFromProperty (robot, jointLimitTableStr);
      for(size_t i=0;i<jointLimitTablesRaw.size();i++){
        // apply margin
        for(size_t j=0;j<jointLimitTablesRaw[i]->lLimitTable().size();j++){
          if(jointLimitTablesRaw[i]->uLimitTable()[j] - jointLimitTablesRaw[i]->lLimitTable()[j] > 0.002){
            jointLimitTablesRaw[i]->uLimitTable()[j] -= 0.001;
            jointLimitTablesRaw[i]->lLimitTable()[j] += 0.001;
          }
        }
        jointLimitTables[jointLimitTablesRaw[i]->getSelfJoint()->jointId()].push_back(jointLimitTablesRaw[i]);
      }
    }

    this->state_.init(robot, jointLimitTables);


  }

  // init modules
  this->resolvedAccelerationController_.init(this->state_);

  return RTC::RTC_OK;
}

// static function
bool ActKinStabilizer::readInPortDataForState(ActKinStabilizer::Ports& ports, const std::string& instance_name, const double& dt,
                                              actkin_stabilizer::State& state){
  bool qAct_updated = false;
  if(ports.m_qActIn_.isNew()) qAct_updated = true;
  while(ports.m_qActIn_.isNew()) ports.m_qActIn_.read();
  while(ports.m_dqActIn_.isNew()) ports.m_dqActIn_.read();
  while(ports.m_actBasePoseIn_.isNew()) ports.m_actBasePoseIn_.read();
  while(ports.m_actBaseVelIn_.isNew()) ports.m_actBaseVelIn_.read();
  state.updateRobotFromIdl(ports.m_qAct_, ports.m_dqAct_, ports.m_actBasePose_, ports.m_actBaseVel_, dt);

  while(ports.m_actContactStateIn_.isNew()) ports.m_actContactStateIn_.read();
  state.updateContactFromIdl(ports.m_actContactState_);

  return qAct_updated;
}

// static function
bool ActKinStabilizer::readInPortDataForGoal(ActKinStabilizer::Ports& ports, const std::string& instance_name, const double& dt, const actkin_stabilizer::State& state,
                                             actkin_stabilizer::Goal& goal){
  if(ports.m_refStateIn_.isNew() || ports.m_refStateUpdatedByService_){
    while(ports.m_refStateIn_.isNew()) ports.m_refStateIn_.read();
    ports.m_refStateUpdatedByService_ = false;
    goal.updateFromIdl(state, ports.m_refState_);
  }
  goal.interpolate(dt);
  return true;
}

// static function
bool ActKinStabilizer::writeOutPortData(const actkin_stabilizer::State& state, const ActKinStabilizer::ControlMode& mode,
                                        ActKinStabilizer::Ports& ports){
  {
    // tau
    ports.m_tau_.tm = ports.m_qAct_.tm;
    ports.m_tau_.data.length(state.robot->numJoints());
    if(mode.now() == ActKinStabilizer::ControlMode::MODE_ST){
      for(int i=0;i<state.robot->numJoints();i++) {
        if(state.jointControllable[i]) {
          double value = state.robot->joint(i)->u();
          if(std::isfinite(value)) {
            ports.m_tau_.data[i] = value;
          }else{
            std::cerr << "m_tau is not finite!" << std::endl;
            ports.m_tau_.data[i] = 0.0;
          }
        }else{
          ports.m_tau_.data[i] = 0.0;
        }
      }
    }else{
      for(int i=0;i<state.robot->numJoints();i++) {
        ports.m_tau_.data[i] = 0.0;
      }
    }
    ports.m_tauOut_.write();
  }

  return true;
}

RTC::ReturnCode_t ActKinStabilizer::onExecute(RTC::UniqueId ec_id){
  std::lock_guard<std::mutex> guard(this->mutex_);

  this->loop_++;
  std::string instance_name = std::string(this->m_profile.instance_name);
  double rate = this->get_context(ec_id)->get_rate();
  if(rate <= 0.0){
    std::cerr << "\x1b[31m[" << instance_name << "] " << " periodic rate is invalid " << rate << "\x1b[39m" << std::endl;
    return RTC::RTC_ERROR;
  }
  double dt = 1.0 / rate;

  // 外部からのサービスコールによって指令されたモード変更を反映する
  this->mode_.update();

  // startST直後の一回のみ実行
  if(this->mode_.isSyncToSTInit()){
    this->state_.onStartStabilizer();
    this->goal_.onStartStabilizer(); // 古いgoalを削除. stopSTしてからstartSTするまでに何らかの形でgoalを与える必要がある.
    this->resolvedAccelerationController_.onStartStabilizer();
  }

  if(this->mode_.isSTRunning()){
    if(!ActKinStabilizer::readInPortDataForState(this->ports_, instance_name, dt,
                                                 this->state_)){
      return RTC::RTC_OK;  // qAct が届かなければ何もしない
    }
    ActKinStabilizer::readInPortDataForGoal(this->ports_, instance_name, dt, this->state_,
                                            this->goal_);

    // goalを満たすようにtauを求めてstate->robot->joint->uに入れる
    this->resolvedAccelerationController_.execResolvedAccelerationController(this->state_, this->goal_, instance_name, dt);
  }

  ActKinStabilizer::writeOutPortData(this->state_, this->mode_,
                                     this->ports_);

  return RTC::RTC_OK;
}

RTC::ReturnCode_t ActKinStabilizer::onActivated(RTC::UniqueId ec_id){
  std::lock_guard<std::mutex> guard(this->mutex_);
  std::cerr << "[" << m_profile.instance_name << "] "<< "onActivated(" << ec_id << ")" << std::endl;
  this->mode_.reset();
  return RTC::RTC_OK;
}
RTC::ReturnCode_t ActKinStabilizer::onDeactivated(RTC::UniqueId ec_id){
  std::lock_guard<std::mutex> guard(this->mutex_);
  std::cerr << "[" << m_profile.instance_name << "] "<< "onDeactivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}
RTC::ReturnCode_t ActKinStabilizer::onFinalize(){ return RTC::RTC_OK; }


bool ActKinStabilizer::startStabilizer(void){
  if(this->mode_.setNextTransition(ControlMode::START_ST)){
    std::cerr << "[" << m_profile.instance_name << "] start ST" << std::endl;
    while (this->mode_.now() != ControlMode::MODE_ST) usleep(1000);
    usleep(1000);
    return true;
  }else{
    std::cerr << "[" << this->m_profile.instance_name << "] already started" << std::endl;
    return false;
  }
}
bool ActKinStabilizer::stopStabilizer(void){
  if(this->mode_.setNextTransition(ControlMode::STOP_ST)){
    std::cerr << "[" << m_profile.instance_name << "] stop ST" << std::endl;
    while (this->mode_.now() != ControlMode::MODE_IDLE) usleep(1000);
    usleep(1000);
    return true;
  }else{
    std::cerr << "[" << this->m_profile.instance_name << "] already stopped" << std::endl;
    return false;
  }
}

bool ActKinStabilizer::setActKinStabilizerParam(const actkin_stabilizer::ActKinStabilizerService::ActKinStabilizerParam& i_param){
  std::lock_guard<std::mutex> guard(this->mutex_);

  // TODO
  return true;
}
bool ActKinStabilizer::getActKinStabilizerParam(actkin_stabilizer::ActKinStabilizerService::ActKinStabilizerParam& i_param) {
  std::lock_guard<std::mutex> guard(this->mutex_);

  // TODO
  return true;
}

bool ActKinStabilizer::setRefState(const actkin_stabilizer_msgs::RefStateIdl& i_param) {
  std::lock_guard<std::mutex> guard(this->mutex_);
  this->ports_.m_refState_ = i_param;
  this->ports_.m_refStateUpdatedByService_ = true;
  return true;
}

bool ActKinStabilizer::getProperty(const std::string& key, std::string& ret) {
  if (this->getProperties().hasKey(key.c_str())) {
    ret = std::string(this->getProperties()[key.c_str()]);
  } else if (this->m_pManager->getConfig().hasKey(key.c_str())) { // 引数 -o で与えたプロパティを捕捉
    ret = std::string(this->m_pManager->getConfig()[key.c_str()]);
  } else {
    return false;
  }
  std::cerr << "[" << this->m_profile.instance_name << "] " << key << ": " << ret <<std::endl;
  return true;
}

extern "C"{
    void ActKinStabilizerInit(RTC::Manager* manager) {
        RTC::Properties profile(ActKinStabilizer_spec);
        manager->registerFactory(profile, RTC::Create<ActKinStabilizer>, RTC::Delete<ActKinStabilizer>);
    }
};

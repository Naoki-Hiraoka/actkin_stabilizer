#include "ActKinStabilizer.h"
#include <cnoid/BodyLoader>
#include <cnoid/ForceSensor>
#include <cnoid/RateGyroSensor>
#include <cnoid/ValueTree>
#include <cnoid/EigenUtil>
#include "MathUtil.h"
#include "RTMUtil.h"
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
  m_qRefIn_("qRef", m_qRef_),
  m_refTauIn_("refTauIn", m_refTau_),
  m_refBasePosIn_("refBasePosIn", m_refBasePos_),
  m_qActIn_("qAct", m_qAct_),
  m_dqActIn_("dqAct", m_dqAct_),
  m_actImuIn_("actImuIn", m_actImu_),
  m_selfCollisionIn_("selfCollisionIn", m_selfCollision_),
  m_primitiveCommandIn_("primitiveCommandIn", m_primitiveCommand_),

  m_genTauOut_("genTauOut", m_genTau_),
  m_actBasePoseOut_("actBasePoseOut", m_actBasePose_),
  m_actBaseTformOut_("actBaseTformOut", m_actBaseTform_),
  m_actBasePosOut_("actBasePosOut", m_actBasePos_),
  m_actBaseRpyOut_("actBaseRpyOut", m_actBaseRpy_),
  m_objectStatesOut_("objectStatesOut", m_objectStates_),
  m_primitiveStateOut_("primitiveStateOut", m_primitiveState_),

  m_actCogOut_("actCogOut", m_actCog_),

  m_ActKinStabilizerServicePort_("ActKinStabilizerService"){
}

ActKinStabilizer::ActKinStabilizer(RTC::Manager* manager) : RTC::DataFlowComponentBase(manager),
  ports_(),
  debugLevel_(0)
{
  this->ports_.m_service0_.setComp(this);
}

RTC::ReturnCode_t ActKinStabilizer::onInitialize(){
  std::cerr << "[" << m_profile.instance_name << "] onInitialize()" << std::endl;

  // add ports
  this->addInPort("qRef", this->ports_.m_qRefIn_);
  this->addInPort("refTauIn", this->ports_.m_refTauIn_);
  this->addInPort("refBasePosIn", this->ports_.m_refBasePosIn_);
  this->addInPort("qAct", this->ports_.m_qActIn_);
  this->addInPort("dqAct", this->ports_.m_dqActIn_);
  this->addInPort("actImuIn", this->ports_.m_actImuIn_);
  this->addInPort("selfCollisionIn", this->ports_.m_selfCollisionIn_);
  this->addInPort("primitiveCommandIn", this->ports_.m_primitiveCommandIn_);
  this->addOutPort("genTauOut", this->ports_.m_genTauOut_);
  this->addOutPort("actBasePoseOut", this->ports_.m_actBasePoseOut_);
  this->addOutPort("actBaseTformOut", this->ports_.m_actBaseTformOut_);
  this->addOutPort("actBasePosOut", this->ports_.m_actBasePosOut_);
  this->addOutPort("actBaseRpyOut", this->ports_.m_actBaseRpyOut_);
  this->addOutPort("objectStatesOut", this->ports_.m_objectStatesOut_);
  this->addOutPort("primitiveStateOut", this->ports_.m_primitiveStateOut_);
  this->addOutPort("actCogOut", this->ports_.m_actCogOut_);
  this->ports_.m_ActKinStabilizerServicePort_.registerProvider("service0", "ActKinStabilizerService", this->ports_.m_service0_);
  this->addPort(this->ports_.m_ActKinStabilizerServicePort_);

  {
    // load robot model
    cnoid::BodyLoader bodyLoader;
    std::string fileName; this->getProperty("model", fileName);
    if (fileName.find("file://") == 0) fileName.erase(0, strlen("file://"));
    cnoid::BodyPtr robot = bodyLoader.load(fileName);
    if(!robot){
      std::cerr << "\x1b[31m[" << this->m_profile.instance_name << "] " << "failed to load model[" << fileName << "]" << "\x1b[39m" << std::endl;
      return RTC::RTC_ERROR;
    }
    if(!robot->rootLink()->isFreeJoint()){
      std::cerr << "\x1b[31m[" << this->m_profile.instance_name << "] " << "rootLink is not FreeJoint [" << fileName << "]" << "\x1b[39m" << std::endl;
      return RTC::RTC_ERROR;
    }
    this->gaitParam_.init(robot);

    // generate JointParams
    for(int i=0;i<this->gaitParam_.robot->body->numJoints();i++){
      cnoid::LinkPtr joint = this->gaitParam_.robot->body->joint(i);
      double climit = 0.0, gearRatio = 0.0, torqueConst = 0.0;
      joint->info()->read("climit",climit); joint->info()->read("gearRatio",gearRatio); joint->info()->read("torqueConst",torqueConst);
      double maxTorque = std::max(climit * gearRatio * torqueConst, 0.0);
      this->gaitParam_.maxTorque[i] = maxTorque;
      this->gaitParam_.softMaxTorque[i].reset(maxTorque);
    }
    std::string jointLimitTableStr; this->getProperty("joint_limit_table",jointLimitTableStr);
    {
      std::vector<std::shared_ptr<joint_limit_table::JointLimitTable> > jointLimitTables = joint_limit_table::readJointLimitTablesFromProperty (this->gaitParam_.robot->body, jointLimitTableStr);
      for(size_t i=0;i<jointLimitTables.size();i++){
        // apply margin
        for(size_t j=0;j<jointLimitTables[i]->lLimitTable().size();j++){
          if(jointLimitTables[i]->uLimitTable()[j] - jointLimitTables[i]->lLimitTable()[j] > 0.002){
            jointLimitTables[i]->uLimitTable()[j] -= 0.001;
            jointLimitTables[i]->lLimitTable()[j] += 0.001;
          }
        }
        this->gaitParam_.jointLimitTables[jointLimitTables[i]->getSelfJoint()->jointId()].push_back(jointLimitTables[i]);
      }
    }

    // apply margin to jointlimit
    for(int i=0;i<this->gaitParam_.robot->body->numJoints();i++){
      cnoid::LinkPtr joint = this->gaitParam_.robot->body->joint(i);
      if(joint->q_upper() - joint->q_lower() > 0.002){
        joint->setJointRange(joint->q_lower()+0.001,joint->q_upper()-0.001);
      }
      // JointVelocityについて. 1.0だと安全.4.0は脚.10.0はlapid manipulation らしい. limitを小さくしすぎた状態で、速い指令を送ると、狭いlimitの中で高優先度タスクを頑張って満たそうとすることで、低優先度タスクを満たす余裕がなくエラーが大きくなってしまうことに注意.
      if(joint->dq_upper() - joint->dq_lower() > 0.02){
        joint->setJointVelocityRange(joint->dq_lower()+0.01,joint->dq_upper()-0.01);
      }
    }
  }

  {
    // setup default tasks.
    //   contactとattentionの初期値として、パラメータend_effectorsを用いる. 無くても良い. (あれば)0,1番目がcontactになり、それ以外がattentionになる. contactが1つ以上あれば、cogとrootもattentionとして加える.
    std::string endEffectors; this->getProperty("end_effectors", endEffectors);
    std::stringstream ss_endEffectors(endEffectors);
    std::string buf;
    int idx = 0;
    while(std::getline(ss_endEffectors, buf, ',')){
      std::string name;
      std::string parentLink;
      cnoid::Vector3 localp;
      cnoid::Vector3 localaxis;
      double localangle;

      //   name, parentLink, (not used), x, y, z, theta, ax, ay, az
      name = buf;
      if(!std::getline(ss_endEffectors, buf, ',')) break; parentLink = buf;
      if(!std::getline(ss_endEffectors, buf, ',')) break; // not used
      if(!std::getline(ss_endEffectors, buf, ',')) break; localp[0] = std::stod(buf);
      if(!std::getline(ss_endEffectors, buf, ',')) break; localp[1] = std::stod(buf);
      if(!std::getline(ss_endEffectors, buf, ',')) break; localp[2] = std::stod(buf);
      if(!std::getline(ss_endEffectors, buf, ',')) break; localaxis[0] = std::stod(buf);
      if(!std::getline(ss_endEffectors, buf, ',')) break; localaxis[1] = std::stod(buf);
      if(!std::getline(ss_endEffectors, buf, ',')) break; localaxis[2] = std::stod(buf);
      if(!std::getline(ss_endEffectors, buf, ',')) break; localangle = std::stod(buf);

      // check validity
      name.erase(std::remove(name.begin(), name.end(), ' '), name.end()); // remove whitespace
      parentLink.erase(std::remove(parentLink.begin(), parentLink.end(), ' '), parentLink.end()); // remove whitespace
      if(!this->gaitParam_.robot->body->link(parentLink)){
        std::cerr << "\x1b[31m[" << this->m_profile.instance_name << "] " << " link [" << parentLink << "]" << " is not found for " << name << "\x1b[39m" << std::endl;
        return RTC::RTC_ERROR;
      }
      cnoid::Matrix3 localR;
      if(localaxis.norm() == 0) localR = cnoid::Matrix3::Identity();
      else localR = Eigen::AngleAxisd(localangle, localaxis.normalized()).toRotationMatrix();
      cnoid::Position localT;
      localT.translation() = localp;
      localT.linear() = localR;

      if(idx<2){
        std::shared_ptr<Contact> contact = std::make_shared<Contact>();
        contact->name = name;
        contact->link1 = this->gaitParam_.robot->body->link(parentLink);
        contact->localPose1 = localT;
        // とりあえず足裏のようなsurface contactで初期化
        contact->S.resize(6,6);
        contact->S.setIdentity();
        // 50<  0  0  1  0  0  0 < 1e10
        // 0 <  1  0 mt  0  0  0 < 1e10
        // 0 < -1  0 mt  0  0  0 < 1e10
        // 0 <  0  1 mt  0  0  0 < 1e10
        // 0 <  0 -1 mt  0  0  0 < 1e10
        // 0 <  0  0 xu  0  1  0 < 1e10
        // 0 <  0  0-xl  0 -1  0 < 1e10
        // 0 <  0  0 yu -1  0  0 < 1e10
        // 0 <  0  0-yl  1  0  0 < 1e10
        // 0 <  0  0 mr  0  0  1 < 1e10
        // 0 <  0  0 mr  0  0 -1 < 1e10
        const double mt=0.5, mr=0.05, xu=0.115, xl=-0.095, yu=0.065, yl=-0.065;
        contact->ld.resize(11); contact->C.resize(11,6); contact->ud.resize(11);
        contact->ld[0] = 50.0; contact->C.insert(0,2) = 1.0; contact->ud[0] = 1e10;
        contact->ld[1] = 0.0; contact->C.insert(1,0) = 1.0; contact->C.insert(1,2) = mt; contact->ud[1] = 1e10;
        contact->ld[2] = 0.0; contact->C.insert(2,0) = -1.0; contact->C.insert(2,2) = mt; contact->ud[2] = 1e10;
        contact->ld[3] = 0.0; contact->C.insert(3,1) = 1.0; contact->C.insert(3,2) = mt; contact->ud[3] = 1e10;
        contact->ld[4] = 0.0; contact->C.insert(4,1) = -1.0; contact->C.insert(4,2) = mt; contact->ud[4] = 1e10;
        contact->ld[5] = 0.0; contact->C.insert(5,2) = xu; contact->C.insert(5,4) = 1.0; contact->ud[5] = 1e10;
        contact->ld[6] = 0.0; contact->C.insert(6,2) = -xl; contact->C.insert(6,4) = -1.0; contact->ud[6] = 1e10;
        contact->ld[7] = 0.0; contact->C.insert(7,2) = yu; contact->C.insert(7,3) = -1.0; contact->ud[7] = 1e10;
        contact->ld[8] = 0.0; contact->C.insert(8,2) = -yl; contact->C.insert(8,3) = 1.0; contact->ud[8] = 1e10;
        contact->ld[9] = 0.0; contact->C.insert(9,0) = mr; contact->C.insert(9,5) = 1.0; contact->ud[9] = 1e10;
        contact->ld[10] = 0.0; contact->C.insert(10,0) = mr; contact->C.insert(10,5) = -1.0; contact->ud[10] = 1e10;
        contact->w.resize(6);
        contact->w << 1e2, 1e2, 1e0, 1e2, 1e2, 1e3;

        this->gaitParam_.contacts[contact->name] = contact;
      }else{
        std::shared_ptr<Attention> attention = std::make_shared<Attention>();
        attention->name = name;
        attention->link1 = this->gaitParam_.robot->body->link(parentLink);
        attention->localPose1.reset(localT);
        // とりあえず6軸拘束で初期化
        attention->C.resize(6,6); attention->C.setIdentity(); attention->ld.reset(cnoid::VectorX::Zero(6), cnoid::VectorX::Zero(6), cnoid::VectorX::Zero(6)); attention->ud.reset(cnoid::VectorX::Zero(6), cnoid::VectorX::Zero(6), cnoid::VectorX::Zero(6));
        attention->Kp.resize(6); attention->Kp<<50, 50, 50, 20, 20, 20; attention->Dp.resize(6); attention->Dp<<10, 10, 10, 10, 10, 10;
        attention->limitp.resize(6); attention->limitp<<5.0, 5.0, 5.0, 5.0, 5.0, 5.0; attention->weightp.resize(6); attention->weightp<<1.0,1.0,1.0,1.0,1.0,1.0;
        attention->Kw.resize(6); attention->Kw.setZero(); attention->Kw.resize(6); attention->Kw.setZero();
        attention->limitw.resize(6); attention->limitw.setZero();
        this->gaitParam_.attentions[attention->name] = attention;
      }
      idx++;
    }

    if(idx>0){
      {
        // cog
        std::shared_ptr<Attention> attention = std::make_shared<Attention>();
        attention->name = "cog";
        attention->cog2 = this->gaitParam_.robot->body;
        attention->C.resize(3,6); for(int i=0;i<3;i++) attention->C.insert(i,i)=1.0; attention->ld.reset(cnoid::VectorX::Zero(3), cnoid::VectorX::Zero(3), cnoid::VectorX::Zero(3)); attention->ud.reset(cnoid::VectorX::Zero(3), cnoid::VectorX::Zero(3), cnoid::VectorX::Zero(3));
        attention->Kp.resize(3); attention->Kp<<50, 50, 50; attention->Dp.resize(3); attention->Dp<<10, 10, 10;
        attention->limitp.resize(3); attention->limitp<<5.0, 5.0, 5.0; attention->weightp.resize(3); attention->weightp<<1.0,1.0,1.0;
        attention->Kw.resize(3); attention->Kw.setZero(); attention->Kw.resize(3); attention->Kw.setZero();
        attention->limitw.resize(3); attention->limitw.setZero();
        // TODO gain, priority
        this->gaitParam_.attentions[attention->name] = attention;
      }
      {
        // root
        std::shared_ptr<Attention> attention = std::make_shared<Attention>();
        attention->name = "root";
        attention->link1 = this->gaitParam_.robot->body->rootLink();
        attention->C = Eigen::SparseMatrix<double,Eigen::RowMajor>(3,6); for(int i=0;i<3;i++) attention->C.insert(i,3+i)=1.0;
        attention->ld.reset(cnoid::VectorX::Zero(3), cnoid::VectorX::Zero(3), cnoid::VectorX::Zero(3)); attention->ud.reset(cnoid::VectorX::Zero(3), cnoid::VectorX::Zero(3), cnoid::VectorX::Zero(3));
        attention->Kp.resize(3); attention->Kp<<100, 100, 100; attention->Dp.resize(3); attention->Dp<<25, 25, 25;
        attention->limitp.resize(3); attention->limitp<<5.0, 5.0, 5.0; attention->weightp.resize(3); attention->weightp<<1.0,1.0,1.0;
        attention->Kw.resize(3); attention->Kw.setZero(); attention->Kw.resize(3); attention->Kw.setZero();
        attention->limitw.resize(3); attention->limitw.setZero();
        // TODO gain, priority
        this->gaitParam_.attentions[attention->name] = attention;
      }

      GaitParam::calcActiveObjectsContacts(this->gaitParam_.robot, this->gaitParam_.objects, this->gaitParam_.contacts, // input
                                           this->gaitParam_.activeObjects, this->gaitParam_.activeContacts); // output
      GaitParam::calcPrioritizedAttentions(this->gaitParam_.attentions, // input
                                           this->gaitParam_.prioritizedAttentions); // output
    }
  }

  // init modules
  this->actToGenFrameConverter_.init(this->gaitParam_);
  this->wrenchDistributor_.init(this->gaitParam_);
  this->resolvedAccelerationController_.init(this->gaitParam_);

  // initialize parameters
  this->loop_ = 0;

  return RTC::RTC_OK;
}

// static function
bool ActKinStabilizer::readInPortData(const double& dt, const GaitParam& gaitParam, const ActKinStabilizer::ControlMode& mode, ActKinStabilizer::Ports& ports, cnoid::BodyPtr refRobotRaw, cnoid::BodyPtr actRobotRaw, std::vector<GaitParam::Collision>& selfCollision, std::unordered_map<std::string, std::shared_ptr<Contact> >& contacts, std::unordered_map<std::string, std::shared_ptr<Attention> >& attentions, std::vector<std::shared_ptr<Object> >& activeObjects, std::vector<std::shared_ptr<Contact> >& activeContacts, std::vector<std::vector<std::shared_ptr<Attention> > >& prioritizedAttentions){
  bool qRef_updated = false;
  if(ports.m_qRefIn_.isNew()){
    ports.m_qRefIn_.read();
    if(ports.m_qRef_.data.length() == refRobotRaw->numJoints()){
      if(!rtmutil::isAllFinite(ports.m_qRef_.data)){
        std::cerr << "m_qRef is not finite!" << std::endl;
      }else{
        qRef_updated = true;
        for(int i=0;i<ports.m_qRef_.data.length();i++){
          double q = ports.m_qRef_.data[i];
          double dq = (q - refRobotRaw->joint(i)->q()) / dt; // 指令関節角度はSequencePlayerなどによって滑らかになっていると仮定し、そのまま微分する. 初回のloopのときに値がとんでしまうが、MODE_IDLE状態なので問題ない
          double ddq = (dq - refRobotRaw->joint(i)->dq()) / dt; // 指令関節角度はSequencePlayerなどによって滑らかになっていると仮定し、そのまま微分する
          refRobotRaw->joint(i)->q() = q;
          refRobotRaw->joint(i)->dq() = dq;
          refRobotRaw->joint(i)->ddq() = ddq;
        }
      }
    }
  }
  if(ports.m_refTauIn_.isNew()){
    ports.m_refTauIn_.read();
    if(ports.m_refTau_.data.length() == refRobotRaw->numJoints()){
      if(!rtmutil::isAllFinite(ports.m_refTau_.data)){
        std::cerr << "m_refTau is not finite!" << std::endl;
      }else{
        for(int i=0;i<ports.m_refTau_.data.length();i++){
          refRobotRaw->joint(i)->u() = ports.m_refTau_.data[i];
        }
      }
    }
  }
  if(ports.m_refBasePosIn_.isNew()){
    ports.m_refBasePosIn_.read();
    if(rtmutil::isAllFinite(ports.m_refBasePos_.data)){
      cnoid::Vector3 p; eigen_rtm_conversions::pointRTMToEigen(ports.m_refBasePos_.data, p);
      cnoid::Vector3 v = (p - refRobotRaw->rootLink()->p()) / dt;  // 指令ルート位置はSequencePlayerなどによって滑らかになっていると仮定し、そのまま微分する. 初回のloopのときに値がとんでしまうが、MODE_IDLE状態なので問題ない
      cnoid::Vector3 dv = (v - refRobotRaw->rootLink()->v()) / dt;  // 指令ルート位置はSequencePlayerなどによって滑らかになっていると仮定し、そのまま微分する
      refRobotRaw->rootLink()->p() = p;
      refRobotRaw->rootLink()->v() = v;
      refRobotRaw->rootLink()->dv() = dv;

      actRobotRaw->rootLink()->p() = p; // actRobotRawの位置をとりあえずrefの値にしておく.
    } else {
      std::cerr << "m_refBasePos is not finite!" << std::endl;
    }
  }
  // refRobotRaw->calcForwardKinematics(); // refRobotRawのFKは不要

  if(ports.m_qActIn_.isNew()){
    ports.m_qActIn_.read();
    if(ports.m_qAct_.data.length() == actRobotRaw->numJoints()){
      if(!rtmutil::isAllFinite(ports.m_qAct_.data)){
        std::cerr << "m_qAct is not finite!" << std::endl;
      }else{
        for(int i=0;i<ports.m_qAct_.data.length();i++){
          double q = ports.m_qAct_.data[i];
          double dq = (q - actRobotRaw->joint(i)->q()) / dt; // そのまま微分する. 平滑化はactRobotで行う. 初回のloopや基板を立ち上げ直したときに値がとんでしまうが、MODE_IDLE状態またはサーボオフ状態なので問題ない
          actRobotRaw->joint(i)->q() = q;
          actRobotRaw->joint(i)->q() = dq;
        }
      }
    }
  }
  if(ports.m_dqActIn_.isNew()){
    ports.m_dqActIn_.read();

    // 制御基板からの関節速度計測値は、エンコーダの値が変わる頻度よりも速い頻度で基板の電流制御をするためにエンコーダの値を平滑化したものなので、必ずしもhrpsysの1制御周期ぶんのエンコーダの値の総変化を表すものではない. そのため、hrpsysのレイヤで関節角度を微分しなおして、その値を関節速度として使う方がよい場合がある.
    // dqActのポートが繋がっていればdqActを使い、つながっていなければqActを微分したものをつかう
    if(ports.m_dqAct_.data.length() == actRobotRaw->numJoints()){
      if(!rtmutil::isAllFinite(ports.m_dqAct_.data)){
        std::cerr << "m_dqAct is not finite!" << std::endl;
      }else{
        for(int i=0;i<ports.m_dqAct_.data.length();i++){
          actRobotRaw->joint(i)->dq() = ports.m_dqAct_.data[i];
        }
      }
    }
  }
  if(ports.m_actImuIn_.isNew()){
    ports.m_actImuIn_.read();
    if(rtmutil::isAllFinite(ports.m_actImu_.data)){
      cnoid::Matrix3 imuR; eigen_rtm_conversions::orientationRTMToEigen(ports.m_actImu_.data, imuR);
      cnoid::RateGyroSensorPtr imu = actRobotRaw->findDevice<cnoid::RateGyroSensor>("gyrometer");
      imu->link()->R() = imuR * imu->R_local().inverse();
    }else{
      std::cerr << "m_actImu is not finite!" << std::endl;
    }
  }
  // actRobotRaw->calcForwardKinematics(); // actRobotRawのFKは不要. むしろimuRが消えてしまうのでしてはいけない


  if(ports.m_selfCollisionIn_.isNew()) {
    ports.m_selfCollisionIn_.read();
    selfCollision.resize(ports.m_selfCollision_.data.length());
    for (int i=0; i<selfCollision.size(); i++){
      if(gaitParam.robot->body->link(std::string(ports.m_selfCollision_.data[i].link1)) &&
         rtmutil::isAllFinite(ports.m_selfCollision_.data[i].point1) &&
         gaitParam.robot->body->link(std::string(ports.m_selfCollision_.data[i].link2)) &&
         rtmutil::isAllFinite(ports.m_selfCollision_.data[i].point2) &&
         rtmutil::isAllFinite(ports.m_selfCollision_.data[i].direction21) &&
         std::isfinite(ports.m_selfCollision_.data[i].distance)){
        selfCollision[i].link1 = gaitParam.robot->body->link(std::string(ports.m_selfCollision_.data[i].link1));
        eigen_rtm_conversions::pointRTMToEigen(ports.m_selfCollision_.data[i].point1, selfCollision[i].point1);
        selfCollision[i].link2 = gaitParam.robot->body->link(std::string(ports.m_selfCollision_.data[i].link2));
        eigen_rtm_conversions::pointRTMToEigen(ports.m_selfCollision_.data[i].point2, selfCollision[i].point2);
        eigen_rtm_conversions::vectorRTMToEigen(ports.m_selfCollision_.data[i].direction21, selfCollision[i].direction21);
        selfCollision[i].distance = ports.m_selfCollision_.data[i].distance;
      }else{
        std::cerr << "m_selfCollision is not finite or has unknown link name!" << std::endl;
        selfCollision.resize(0);
        break;
      }
    }
  }


  if(ports.m_primitiveCommandIn_.isNew()){
    ports.m_primitiveCommandIn_.read();

    bool contacts_changed = false;
    for(int i=0;i<ports.m_primitiveCommand_.contactParams.length();i++){
      actkin_stabilizer::ContactParamIdl& param = ports.m_primitiveCommand_.contactParams[i];
      std::unordered_map<std::string, std::shared_ptr<Contact> >::iterator it = contacts.find(std::string(param.name));
      if(((it == contacts.end()) && (param.remove == false)) ||
         ((it != contacts.end()) && (param.remove == false) && (param.stateId == it->second->stateId))){
        // contactを上書き or 新規追加
        std::shared_ptr<Contact> contact = std::make_shared<Contact>();
        if(!contact->initializeFromIdl(gaitParam.robot, gaitParam.objects, param)) continue;
        if(mode.isABCRunning()) contact->onStartAutoBalancer();
        if(mode.isSTRunning()) contact->onStartStabilizer();
        contacts[contact->name] = contact;
        contacts_changed = true;
      }else if((it != contacts.end()) && (param.remove == true) && (param.stateId == it->second->stateId)){
        // contactを消去
        contacts.erase(it);
        contacts_changed = true;
      }
    }
    if(contacts_changed){
      GaitParam::calcActiveObjectsContacts(gaitParam.robot, gaitParam.objects, contacts, // input
                                           activeObjects, activeContacts); // output
    }

    bool attentions_changed = false;
    for(int i=0;i<ports.m_primitiveCommand_.attentionParams.length();i++){
      actkin_stabilizer::AttentionParamIdl& param = ports.m_primitiveCommand_.attentionParams[i];
      std::unordered_map<std::string, std::shared_ptr<Attention> >::iterator it = attentions.find(std::string(param.name));
      if(((it == attentions.end()) && (param.remove == false)) ||
         ((it != attentions.end()) && (param.remove == false) && (param.stateId == it->second->stateId))){
        // attentionを上書き or 新規追加
        std::shared_ptr<Attention> attention = std::make_shared<Attention>();
        if(!attention->initializeFromIdl(gaitParam.robot, gaitParam.objects, param)) continue;
        if(mode.isABCRunning()) attention->onStartAutoBalancer();
        if(mode.isSTRunning()) attention->onStartStabilizer();
        attentions[attention->name] = attention;
        attentions_changed = true;
      }else if((it != attentions.end()) && (param.remove == true) && (param.stateId == it->second->stateId)){
        // attentionを消去
        attentions.erase(it);
        attentions_changed = true;
      }
    }
    if(attentions_changed){
      GaitParam::calcPrioritizedAttentions(attentions, // input
                                           prioritizedAttentions); // output
    }

    for(int i=0;i<ports.m_primitiveCommand_.attentionDatas.length();i++){
      actkin_stabilizer::AttentionDataIdl& param = ports.m_primitiveCommand_.attentionDatas[i];
      std::unordered_map<std::string, std::shared_ptr<Attention> >::iterator it = attentions.find(std::string(param.name));
      if((it != attentions.end()) && (param.stateId == it->second->stateId)){
        // attentionを上書き
        it->second->updateFromIdl(param);
      }
    }
  }

  return qRef_updated;
}

// static function
bool ActKinStabilizer::writeOutPortData(ActKinStabilizer::Ports& ports, const ActKinStabilizer::ControlMode& mode, double dt, const GaitParam& gaitParam, cpp_filters::TwoPointInterpolatorSE3& outputRootPoseFilter){
  {
    // tau
    ports.m_genTau_.tm = ports.m_qRef_.tm;
    ports.m_genTau_.data.length(gaitParam.robot->body->numJoints());
    for(int i=0;i<gaitParam.robot->body->numJoints();i++){
      double value = 0.0;
      if(mode.now() == ActKinStabilizer::ControlMode::MODE_IDLE || mode.now() == ActKinStabilizer::ControlMode::MODE_SYNC_TO_ABC || mode.now() == ActKinStabilizer::ControlMode::MODE_ABC || mode.now() == ActKinStabilizer::ControlMode::MODE_SYNC_TO_IDLE || !gaitParam.jointControllable[i]){
        value = gaitParam.refRobotRaw->joint(i)->u();
      }else if(mode.now() == ActKinStabilizer::ControlMode::MODE_SYNC_TO_STOPST){
        value = gaitParam.refRobotRaw->joint(i)->u(); // stopST補間中は、tauを少しずつ0に減らして行きたくなるが、吹っ飛ぶことがあるので一気に0にした方が安全
      }else if(mode.now() == ActKinStabilizer::ControlMode::MODE_SYNC_TO_ST || mode.now() == ActKinStabilizer::ControlMode::MODE_ST){
        value = gaitParam.robot->body->joint(i)->u();
      }

      if(std::isfinite(value)) ports.m_genTau_.data[i] = value;
      else std::cerr << "m_genTau is not finite!" << std::endl;
    }
    ports.m_genTauOut_.write();
  }

  {
    // basePose (hrpsys_odom)
    if(mode.now() == ActKinStabilizer::ControlMode::MODE_IDLE){
      outputRootPoseFilter.reset(gaitParam.actRobotRaw->rootLink()->T()); // pはref値. RはIMUが入っている
    }else if(mode.now() == ActKinStabilizer::ControlMode::MODE_SYNC_TO_IDLE){
      outputRootPoseFilter.setGoal(gaitParam.actRobotRaw->rootLink()->T(), mode.remainTime());
    }else{
      outputRootPoseFilter.reset(gaitParam.robot->body->rootLink()->T()); // 積算されていく
    }
    outputRootPoseFilter.interpolate(dt);
    cnoid::Position basePose = outputRootPoseFilter.value();

    if(basePose.translation().allFinite() && basePose.linear().isUnitary()){
      ports.m_actBasePose_.tm = ports.m_qRef_.tm;
      eigen_rtm_conversions::poseEigenToRTM(basePose,ports.m_actBasePose_.data);
      ports.m_actBasePoseOut_.write();

      cnoid::Vector3 basePos = basePose.translation();
      cnoid::Matrix3 baseR = basePose.linear();
      ports.m_actBaseTform_.tm = ports.m_qRef_.tm;
      ports.m_actBaseTform_.data.length(12);
      for(int i=0;i<3;i++){
        ports.m_actBaseTform_.data[i] = basePos[i];
      }
      for(int i=0;i<3;i++){
        for(int j=0;j<3;j++){
          ports.m_actBaseTform_.data[3+i*3+j] = baseR(i,j);// row major
        }
      }
      ports.m_actBaseTformOut_.write();

      ports.m_actBasePos_.tm = ports.m_qRef_.tm;
      eigen_rtm_conversions::pointEigenToRTM(basePos,ports.m_actBasePos_.data);
      ports.m_actBasePosOut_.write();
      ports.m_actBaseRpy_.tm = ports.m_qRef_.tm;
      eigen_rtm_conversions::orientationEigenToRTM(baseR,ports.m_actBaseRpy_.data);
      ports.m_actBaseRpyOut_.write();
    }else{
      std::cerr << "m_actBasePose is not finite!" << std::endl;
    }
  }


  {
    // objectStates
    ports.m_objectStates_.length(gaitParam.objects.size());
    int idx = 0;
    bool isFinite = true;
    for(std::unordered_map<std::string, std::shared_ptr<Object> >::const_iterator it=gaitParam.objects.begin(); it!=gaitParam.objects.end() && isFinite;it++){
      cnoid::BodyPtr body = it->second->body;
      ports.m_objectStates_[idx].tm = ports.m_qRef_.tm;
      ports.m_objectStates_[idx].name = it->second->name.c_str();
      if(!(body->rootLink()->T().translation().allFinite() && body->rootLink()->T().linear().isUnitary())){
        std::cerr << "object pose is not finite! " << it->second->name << std::endl;
        isFinite = false;
      }
      eigen_rtm_conversions::poseEigenToRTM(body->rootLink()->T(), ports.m_objectStates_[idx].pose);
      ports.m_objectStates_[idx].q.length(body->numJoints());
      for(int i=0;i<body->numJoints();i++){
        if(!std::isfinite(body->joint(i)->q())){
          std::cerr << "object joint is not finite! " << it->second->name << std::endl;
          isFinite = false;
          break;
        }
        ports.m_objectStates_[idx].q[i] = body->joint(i)->q();
      }
      idx++;
    }
    if(isFinite) ports.m_objectStatesOut_.write();
  }

  {
    // primitiveState
    ports.m_primitiveState_.tm = ports.m_qRef_.tm;
    ports.m_primitiveState_.contactParams.length(gaitParam.contacts.size());
    int idx = 0;
    for(std::unordered_map<std::string, std::shared_ptr<Contact> >::const_iterator it=gaitParam.contacts.begin(); it!=gaitParam.contacts.end(); it++){
      it->second->copyToIdl(ports.m_primitiveState_.contactParams[idx]);
      idx++;
    }
    idx = 0;
    ports.m_primitiveState_.attentionParams.length(gaitParam.attentions.size());
    for(std::unordered_map<std::string, std::shared_ptr<Attention> >::const_iterator it=gaitParam.attentions.begin(); it!=gaitParam.attentions.end(); it++){
      it->second->copyToIdl(ports.m_primitiveState_.attentionParams[idx]);
      idx++;
    }
    ports.m_primitiveStateOut_.write();
  }

  // only for logger. (IDLE時の出力や、モード遷移時の連続性はてきとうで良い)
  if(mode.isABCRunning()){
    ports.m_actCog_.tm = ports.m_qRef_.tm;
    eigen_rtm_conversions::pointEigenToRTM(gaitParam.robot->body->centerOfMass(), ports.m_actCog_.data);
    ports.m_actCogOut_.write();
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

  // 内部の補間器をdtだけ進める
  this->mode_.update(dt);
  this->gaitParam_.onExecute(dt);
  this->actToGenFrameConverter_.onExecute(dt);
  this->resolvedAccelerationController_.onExecute(dt);
  this->wrenchDistributor_.onExecute(dt);


  if(!ActKinStabilizer::readInPortData(dt, this->gaitParam_, this->mode_, this->ports_, this->gaitParam_.refRobotRaw, this->gaitParam_.actRobotRaw, this->gaitParam_.selfCollision, this->gaitParam_.contacts, this->gaitParam_.attentions, this->gaitParam_.activeObjects, this->gaitParam_.activeContacts, this->gaitParam_.prioritizedAttentions)){
    return RTC::RTC_OK;  // qRef が届かなければ何もしない
  }

  if(this->mode_.isABCRunning()) {
    if(this->mode_.isSyncToABCInit()){ // startAutoBalancer直後の初回. 内部パラメータのリセット
      this->gaitParam_.onStartAutoBalancer();
      this->actToGenFrameConverter_.onStartAutoBalancer();
    }

    // robotとobjectsの位置姿勢と速度を更新する. actRobotRawとcontactsを用いて
    this->actToGenFrameConverter_.convertFrame(this->gaitParam_, dt, // input
                                               this->gaitParam_.robot, this->gaitParam_.activeObjects); // input & output

    if(this->mode_.isSTRunning()){
      if(this->mode_.isSyncToSTInit()){ // startStabilizer直後の初回. 内部パラメータのリセット
        this->gaitParam_.onStartStabilizer();
        this->resolvedAccelerationController_.onStartStabilizer();
        this->wrenchDistributor_.onStartStabilizer();
      }

      // contactsとattentionsを満たすようにrobotとobjectsの目標加速度を求める
      this->resolvedAccelerationController_.execResolvedAccelerationController(this->gaitParam_, dt,
                                                                               this->gaitParam_.robot, this->gaitParam_.activeObjects);

      // contactsに接触力を分配し、robotの目標関節トルクを求める
      this->wrenchDistributor_.execWrenchDistributor(this->gaitParam_, dt,
                                                     this->gaitParam_.robot);
    }
  }

  ActKinStabilizer::writeOutPortData(this->ports_, this->mode_, dt, this->gaitParam_, this->outputRootPoseFilter_);

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




bool ActKinStabilizer::startAutoBalancer(){
  if(this->mode_.setNextTransition(ControlMode::START_ABC)){
    std::cerr << "[" << m_profile.instance_name << "] start auto balancer mode" << std::endl;
    while (this->mode_.now() != ControlMode::MODE_ABC) usleep(1000);
    usleep(1000);
    return true;
  }else{
    std::cerr << "[" << this->m_profile.instance_name << "] auto balancer is already started" << std::endl;
    return false;
  }
}
bool ActKinStabilizer::stopAutoBalancer(){
  if(this->mode_.setNextTransition(ControlMode::STOP_ABC)){
    std::cerr << "[" << m_profile.instance_name << "] stop auto balancer mode" << std::endl;
    while (this->mode_.now() != ControlMode::MODE_IDLE) usleep(1000);
    usleep(1000);
    return true;
  }else{
    std::cerr << "[" << this->m_profile.instance_name << "] auto balancer is already stopped or stabilizer is running" << std::endl;
    return false;
  }
}
bool ActKinStabilizer::startStabilizer(void){
  if(this->mode_.setNextTransition(ControlMode::START_ST)){
    std::cerr << "[" << m_profile.instance_name << "] start ST" << std::endl;
    while (this->mode_.now() != ControlMode::MODE_ST) usleep(1000);
    usleep(1000);
    return true;
  }else{
    std::cerr << "[" << this->m_profile.instance_name << "] Please start AutoBalancer" << std::endl;
    return false;
  }
}
bool ActKinStabilizer::stopStabilizer(void){
  if(this->mode_.setNextTransition(ControlMode::STOP_ST)){
    std::cerr << "[" << m_profile.instance_name << "] stop ST" << std::endl;
    while (this->mode_.now() != ControlMode::MODE_ABC) usleep(1000);
    usleep(1000);
    return true;
  }else{
    std::cerr << "[" << this->m_profile.instance_name << "] Please start AutoBalancer" << std::endl;
    return false;
  }
}

bool ActKinStabilizer::setPrimitiveState(const actkin_stabilizer::PrimitiveStateIdl& command){
  return true;
}
bool ActKinStabilizer::getPrimitiveState(actkin_stabilizer::PrimitiveStateIdl& command){
  return true;
}
bool ActKinStabilizer::resetPrimitiveState(const actkin_stabilizer::PrimitiveStateIdl& command){
  return true;
}
bool ActKinStabilizer::goActual(){
  return true;
}
bool ActKinStabilizer::loadObject(const std::string& name, const std::string& file){
  return true;
}
bool ActKinStabilizer::unloadObject(const std::string& name){
  return true;
}
bool ActKinStabilizer::setObjectState(const actkin_stabilizer::ObjectStateIdl& obj){
  return true;
}
bool ActKinStabilizer::setObjectStates(const actkin_stabilizer::ObjectStateIdlSeq& objs){
  return true;
}
bool ActKinStabilizer::getObjectStates(actkin_stabilizer::ObjectStateIdlSeq& objs){
  return true;
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

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
        std::shared_ptr<Contact> contact;
        contact->name = name;
        contact->link1 = this->gaitParam_.robot->body->link(parentLink);
        contact->localPose1 = localT;
        this->gaitParam_.contacts[contact->name] = contact;
      }else{
        std::shared_ptr<Attention> attention;
        attention->name = name;
        attention->link1 = this->gaitParam_.robot->body->link(parentLink);
        attention->localPose1.reset(localT);
        this->gaitParam_.attentions[attention->name] = attention;
      }
      idx++;
    }

    if(idx>0){
      {
        // cog
        std::shared_ptr<Attention> attention;
        attention->name = "cog";
        attention->cog2 = this->gaitParam_.robot->body;
        // TODO gain, priority
        this->gaitParam_.attentions[attention->name] = attention;
      }
      {
        // root
        std::shared_ptr<Attention> attention;
        attention->name = "root";
        attention->link1 = this->gaitParam_.robot->body->rootLink();
        attention->C = Eigen::SparseMatrix<double,Eigen::RowMajor>(3,6); for(int i=0;i<3;i++) attention->C.insert(3+i,i)=1.0;
        attention->ld.resize(3); attention->ld.setZero(); attention->ud.resize(3); attention->ud.setZero();
        attention->Kp.resize(3); attention->Kp<<100, 100, 100; attention->Dp.resize(3); attention->Dp<<25, 25, 25;
        attention->limitp.resize(3); attention->limitp<<5.0, 5.0, 5.0; attention->weightp.resize(3); attention->weightp<<1.0,1.0,1.0;
        attention->Kw.resize(3); attention->Kw.setZero(); attention->Kw.resize(3); attention->Kw.setZero();
        attention->limitw.resize(3); attention->limitw.setZero();
        // TODO gain, priority
        this->gaitParam_.attentions[attention->name] = attention;
      }
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
      actRobotRaw->calcForwardKinematics();
      cnoid::RateGyroSensorPtr imu = actRobotRaw->findDevice<cnoid::RateGyroSensor>("gyrometer");
      cnoid::Matrix3 imuR = imu->link()->R() * imu->R_local();
      cnoid::Matrix3 actR; eigen_rtm_conversions::orientationRTMToEigen(ports.m_actImu_.data, actR);
      actRobotRaw->rootLink()->R() = Eigen::Matrix3d(Eigen::AngleAxisd(actR) * Eigen::AngleAxisd(imuR.transpose() * actRobotRaw->rootLink()->R())); // 単純に3x3行列の空間でRを積算していると、だんだん数値誤差によって回転行列でなくなってしまう恐れがあるので念の為AngleAxisdの空間を介す
    }else{
      std::cerr << "m_actImu is not finite!" << std::endl;
    }
  }
  // actRobotRaw->calcForwardKinematics(); // actRobotRawのFKは不要


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
        std::shared_ptr<Contact> contact;
        contact->name = param.name;
        contact->stateId = param.stateId + 1; // 1 増える
        if(std::string(param.obj1) == ""){
          if(std::string(param.link1) == ""){
            contact->link1 = nullptr; // world
          }else{
            cnoid::LinkPtr link = gaitParam.robot->body->link(std::string(param.link1));
            if(link) {
              contact->link1 = link; // robotのlink
            }else{
              std::cerr << "contact link is not found! " << param.link1 << std::endl;
              continue;
            }
          }
        }else{
          std::unordered_map<std::string, std::shared_ptr<Object> >::const_iterator object_it = gaitParam.objects.find(std::string(param.obj1));
          if(object_it != gaitParam.objects.end()){
            cnoid::LinkPtr link = object_it->second->body->link(std::string(param.link1));
            if(link) {
              contact->link1 = link; // objectのlink
            }else{
              std::cerr << "contact link is not found! " << param.link1 << std::endl;
              continue;
            }
          }else{
            std::cerr << "contact object is not found! " << param.obj1 << std::endl;
            continue;
          }
        }
        if(!rtmutil::isAllFinite(param.localPose1)){
          std::cerr << "contact localPose1 is not finite! " << std::endl;
          continue;
        }
        eigen_rtm_conversions::poseRTMToEigen(param.localPose1, contact->localPose1);
        if(std::string(param.obj2) == ""){
          if(std::string(param.link2) == ""){
            contact->link2 = nullptr; // world
          }else{
            cnoid::LinkPtr link = gaitParam.robot->body->link(std::string(param.link2));
            if(link) {
              contact->link2 = link; // robotのlink
            }else{
              std::cerr << "contact link is not found! " << param.link2 << std::endl;
              continue;
            }
          }
        }else{
          std::unordered_map<std::string, std::shared_ptr<Object> >::const_iterator object_it = gaitParam.objects.find(std::string(param.obj2));
          if(object_it != gaitParam.objects.end()){
            cnoid::LinkPtr link = object_it->second->body->link(std::string(param.link2));
            if(link) {
              contact->link2 = link; // objectのlink
            }else{
              std::cerr << "contact link is not found! " << param.link2 << std::endl;
              continue;
            }
          }else{
            std::cerr << "contact object is not found! " << param.obj2 << std::endl;
            continue;
          }
        }
        int axis = 0; for(int a=0;a<param.axis.length();a++) if(param.axis[i]) axis++;
        contact->S = Eigen::SparseMatrix<double,Eigen::RowMajor>(axis,6);
        axis = 0;
        for(int a=0;a<param.axis.length();a++) {
          if(param.axis[i]) {
            contact->S.insert(axis,i) = 1.0;
            axis++;
          }
        }
        if(!rtmutil::isAllFinite(param.C)){ std::cerr << "contact C is not finite! " << std::endl; continue; }
        bool valid = true;
        for(int row = 0; row < param.C.length(); row++){
          if(param.C[row].length() != axis) { valid = false; break; }
        }
        if(!valid) { std::cerr << "contact C dimension mismatch! " << std::endl; continue; }
        contact->C = Eigen::SparseMatrix<double,Eigen::RowMajor>(param.C.length(),axis);
        for(int row = 0; row < param.C.length(); row++){
          for(int col = 0; col < param.C[row].length(); col++){
            if(param.C[row][col] != 0.0) {
              contact->C.insert(row,col) = param.C[row][col];
            }
          }
        }
        if(!rtmutil::isAllFinite(param.ld) || !rtmutil::isAllFinite(param.ud)){ std::cerr << "contact ld/ud is not finite! " << std::endl; continue; }
        if(param.ld.length() != param.C.length() || param.ud.length() != param.C.length()){ std::cerr << "contact ld/ud dimension mismatch! " << param.ld.length() << " " << param.ud.length() << std::endl; continue; }
        eigen_rtm_conversions::vectorRTMToEigen(param.ld, contact->ld);
        eigen_rtm_conversions::vectorRTMToEigen(param.ud, contact->ud);
        if(!rtmutil::isAllFinite(param.w)){ std::cerr << "contact w is not finite! " << std::endl; continue; }
        if(param.w.length() != axis){ std::cerr << "contact w dimension mismatch! " << param.w.length() << std::endl; continue; }
        eigen_rtm_conversions::vectorRTMToEigen(param.w, contact->w);
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
      activeObjects.clear();
      activeContacts.clear();
      std::unordered_set<cnoid::BodyPtr> activeBodies; // robotとcontactを介してつながっているobjects.
      std::vector<std::shared_ptr<Contact> > tmpContacts; tmpContacts.reserve(contacts.size());
      std::vector<std::shared_ptr<Contact> > nextTmpContacts; nextTmpContacts.reserve(contacts.size());
      for(std::unordered_map<std::string, std::shared_ptr<Contact> >::iterator it = contacts.begin(); it != contacts.end(); it++) nextTmpContacts.push_back(it->second);
      while(tmpContacts.size() != nextTmpContacts.size()){
        tmpContacts = nextTmpContacts;
        nextTmpContacts.clear();
        for(int i=0;i<tmpContacts.size();i++){
          if(tmpContacts[i]->link1 &&
             ((tmpContacts[i]->link1->body() == gaitParam.robot->body) || (activeBodies.find(tmpContacts[i]->link1->body()) != activeBodies.end()))){
            if(tmpContacts[i]->link2 && tmpContacts[i]->link2->body() != gaitParam.robot->body) activeBodies.insert(tmpContacts[i]->link2->body());
            activeContacts.push_back(tmpContacts[i]);
          }else if(tmpContacts[i]->link2 &&
                   ((tmpContacts[i]->link2->body() == gaitParam.robot->body) || (activeBodies.find(tmpContacts[i]->link2->body()) != activeBodies.end()))){
            if(tmpContacts[i]->link1 && tmpContacts[i]->link1->body() != gaitParam.robot->body) activeBodies.insert(tmpContacts[i]->link1->body());
          }else{
            nextTmpContacts.push_back(tmpContacts[i]);
          }
        }
      }
      for(std::unordered_map<std::string, std::shared_ptr<Object> >::const_iterator it = gaitParam.objects.begin(); it != gaitParam.objects.end(); it++){
        if(activeBodies.find(it->second->body) != activeBodies.end()) {
          activeObjects.push_back(it->second);
        }
      }
    }

    bool attentions_changed = false;
    for(int i=0;i<ports.m_primitiveCommand_.attentionParams.length();i++){
      actkin_stabilizer::AttentionParamIdl& param = ports.m_primitiveCommand_.attentionParams[i];
      std::unordered_map<std::string, std::shared_ptr<Attention> >::iterator it = attentions.find(std::string(param.name));
      if(((it == attentions.end()) && (param.remove == false)) ||
         ((it != attentions.end()) && (param.remove == false) && (param.stateId == it->second->stateId))){
        // attentionを上書き or 新規追加
        std::shared_ptr<Attention> attention;
        attention->name = param.name;
        attention->stateId = param.stateId + 1; // 1 増える
        if(std::string(param.obj1) == ""){
          if(std::string(param.link1) == ""){
            attention->cog1 = nullptr;
            attention->link1 = nullptr; // world
          }else if(std::string(param.link1) == "CM"){
            attention->cog1 = gaitParam.robot->body;
            attention->link1 = nullptr; // robotの重心
          }else{
            cnoid::LinkPtr link = gaitParam.robot->body->link(std::string(param.link1));
            if(link) {
              attention->cog1 = nullptr;
              attention->link1 = link; // robotのlink
            }else{
              std::cerr << "attention link is not found! " << param.link1 << std::endl;
              continue;
            }
          }
        }else{
          std::unordered_map<std::string, std::shared_ptr<Object> >::const_iterator object_it = gaitParam.objects.find(std::string(param.obj1));
          if(object_it != gaitParam.objects.end()){
            if(std::string(param.link1) == "CM"){
              attention->cog1 = object_it->second->body;
              attention->link1 = nullptr; // objectの重心
            }else{
              cnoid::LinkPtr link = object_it->second->body->link(std::string(param.link1));
              if(link) {
                attention->cog1 = nullptr;
                attention->link1 = link; // objectのlink
              }else{
                std::cerr << "attention link is not found! " << param.link1 << std::endl;
                continue;
              }
            }
          }else{
            std::cerr << "attention object is not found! " << param.obj1 << std::endl;
            continue;
          }
        }
        if(!rtmutil::isAllFinite(param.localPose1)){
          std::cerr << "attention localPose1 is not finite! " << std::endl;
          continue;
        }
        cnoid::Position localPose1;
        eigen_rtm_conversions::poseRTMToEigen(param.localPose1, localPose1);
        attention->localPose1.reset(localPose1);
        if(std::string(param.obj2) == ""){
          if(std::string(param.link2) == ""){
            attention->cog2 = nullptr;
            attention->link2 = nullptr; // world
          }else if(std::string(param.link2) == "CM"){
            attention->cog2 = gaitParam.robot->body;
            attention->link2 = nullptr; // robotの重心
          }else{
            cnoid::LinkPtr link = gaitParam.robot->body->link(std::string(param.link2));
            if(link) {
              attention->cog2 = nullptr;
              attention->link2 = link; // robotのlink
            }else{
              std::cerr << "attention link is not found! " << param.link2 << std::endl;
              continue;
            }
          }
        }else{
          std::unordered_map<std::string, std::shared_ptr<Object> >::const_iterator object_it = gaitParam.objects.find(std::string(param.obj2));
          if(object_it != gaitParam.objects.end()){
            if(std::string(param.link2) == "CM"){
              attention->cog2 = object_it->second->body;
              attention->link2 = nullptr; // objectの重心
            }else{
              cnoid::LinkPtr link = object_it->second->body->link(std::string(param.link2));
              if(link) {
                attention->cog2 = nullptr;
                attention->link2 = link; // objectのlink
              }else{
                std::cerr << "attention link is not found! " << param.link2 << std::endl;
                continue;
              }
            }
          }else{
            std::cerr << "attention object is not found! " << param.obj2 << std::endl;
            continue;
          }
        }
        if(!rtmutil::isAllFinite(param.localPose2)){ std::cerr << "attention localPose2 is not finite! " << std::endl; continue; }
        cnoid::Position localPose2;
        eigen_rtm_conversions::poseRTMToEigen(param.localPose2, localPose2);
        attention->localPose2.reset(localPose2);
        if(!rtmutil::isAllFinite(param.refWrench)){ std::cerr << "attention refWrench is not finite! " << std::endl; continue; }
        if(param.refWrench.length() != 6){ std::cerr << "attention refWrench dimension mismatch! " << param.refWrench.length() << std::endl; continue; }
        cnoid::Vector6 refWrench;
        eigen_rtm_conversions::vectorRTMToEigen(param.refWrench,refWrench);
        attention->refWrench.reset(refWrench);
        if(!rtmutil::isAllFinite(param.C)){ std::cerr << "attention C is not finite! " << std::endl; continue; }
        bool valid = true;
        for(int row = 0; row < param.C.length(); row++){
          if(param.C[row].length() != 6) { valid = false; break; }
        }
        if(!valid) { std::cerr << "attention C dimension mismatch! " << std::endl; continue; }
        attention->C = Eigen::SparseMatrix<double,Eigen::RowMajor>(param.C.length(),6);
        for(int row = 0; row < param.C.length(); row++){
          for(int col = 0; col < param.C[row].length(); col++){
            if(param.C[row][col] != 0.0) {
              attention->C.insert(row,col) = param.C[row][col];
            }
          }
        }
        if(!rtmutil::isAllFinite(param.ld) || !rtmutil::isAllFinite(param.ud)){ std::cerr << "attention ld/ud is not finite! " << std::endl; continue; }
        if(param.ld.length() != param.C.length() || param.ud.length() != param.C.length()){ std::cerr << "attention ld/ud dimension mismatch! " << param.ld.length() << " " << param.ud.length() << std::endl; continue; }
        eigen_rtm_conversions::vectorRTMToEigen(param.ld, attention->ld);
        eigen_rtm_conversions::vectorRTMToEigen(param.ud, attention->ud);
        if(!rtmutil::isAllFinite(param.Kp) || !rtmutil::isAllFinite(param.Dp)){ std::cerr << "attention Kp/Dp is not finite! " << std::endl; continue; }
        if(param.Kp.length() != param.C.length() || param.Dp.length() != param.C.length()){ std::cerr << "attention Kp/Dp dimension mismatch! " << param.Kp.length() << " " << param.Dp.length() << std::endl; continue; }
        eigen_rtm_conversions::vectorRTMToEigen(param.Kp, attention->Kp);
        eigen_rtm_conversions::vectorRTMToEigen(param.Dp, attention->Dp);
        if(!rtmutil::isAllFinite(param.limitp)){ std::cerr << "attention limitp is not finite! " << std::endl; continue; }
        if(param.limitp.length() != param.C.length()){ std::cerr << "attention limitp dimension mismatch! " << param.limitp.length() << std::endl; continue; }
        eigen_rtm_conversions::vectorRTMToEigen(param.limitp, attention->limitp);
        if(!rtmutil::isAllFinite(param.weightp)){ std::cerr << "attention weightp is not finite! " << std::endl; continue; }
        if(param.weightp.length() != param.C.length()){ std::cerr << "attention weightp dimension mismatch! " << param.weightp.length() << std::endl; continue; }
        eigen_rtm_conversions::vectorRTMToEigen(param.weightp, attention->weightp);
        attention->priority = param.priority;
        attention->horizon = param.horizon;
        if(!rtmutil::isAllFinite(param.Kw) || !rtmutil::isAllFinite(param.Dw)){ std::cerr << "attention Kw/Dw is not finite! " << std::endl; continue; }
        if(param.Kw.length() != param.C.length() || param.Dw.length() != param.C.length()){ std::cerr << "attention Kw/Dw dimension mismatch! " << param.Kw.length() << " " << param.Dw.length() << std::endl; continue; }
        eigen_rtm_conversions::vectorRTMToEigen(param.Kw, attention->Kw);
        eigen_rtm_conversions::vectorRTMToEigen(param.Dw, attention->Dw);
        if(!rtmutil::isAllFinite(param.limitw)){ std::cerr << "attention limitw is not finite! " << std::endl; continue; }
        if(param.limitw.length() != param.C.length()){ std::cerr << "attention limitw dimension mismatch! " << param.limitw.length() << std::endl; continue; }
        eigen_rtm_conversions::vectorRTMToEigen(param.limitw, attention->limitw);
        if(mode.isABCRunning()) attention->onStartAutoBalancer();
        if(mode.isSTRunning()) attention->onStartStabilizer();
        attentions[attention->name] = attention;
        attentions_changed = true;
      }else if((it != attentions.end()) && (param.remove == true) && (param.stateId == it->second->stateId)){
        // attentoniを消去
        attentions.erase(it);
        attentions_changed = true;
      }
    }
    if(attentions_changed){
      prioritizedAttentions.clear();
      std::vector<std::shared_ptr<Attention> > tmpAttentions; tmpAttentions.reserve(attentions.size());
      std::vector<std::shared_ptr<Attention> > nextTmpAttentions; nextTmpAttentions.reserve(attentions.size());
      for(std::unordered_map<std::string, std::shared_ptr<Attention> >::iterator it = attentions.begin(); it != attentions.end(); it++) tmpAttentions.push_back(it->second);
      while(tmpAttentions.size() > 0){
        long maxPriority = std::numeric_limits<long>::min();
        for(int i=0;i<tmpAttentions.size();i++) {
          if(tmpAttentions[i]->priority > maxPriority) maxPriority = tmpAttentions[i]->priority;
        }
        prioritizedAttentions.emplace_back();
        nextTmpAttentions.clear();
        for(int i=0;i<tmpAttentions.size();i++) {
          if(tmpAttentions[i]->priority == maxPriority) prioritizedAttentions.back().push_back(tmpAttentions[i]);
          else nextTmpAttentions.push_back(tmpAttentions[i]);
        }
        tmpAttentions = nextTmpAttentions;
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

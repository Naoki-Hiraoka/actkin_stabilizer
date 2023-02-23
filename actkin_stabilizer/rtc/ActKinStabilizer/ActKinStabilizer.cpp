#include "ActKinStabilizer.h"
#include <cnoid/BodyLoader>
#include <cnoid/ForceSensor>
#include <cnoid/RateGyroSensor>
#include <cnoid/ValueTree>
#include <cnoid/EigenUtil>
#include "MathUtil.h"
#include "CnoidBodyUtil.h"
#include <limits>

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

  m_genTauOut_("genTauOut", m_genTau_),
  m_actBasePoseOut_("actBasePoseOut", m_actBasePose_),
  m_actBaseTformOut_("actBaseTformOut", m_actBaseTform_),

  m_actBasePosOut_("actBasePosOut", m_actBasePos_),
  m_actBaseRpyOut_("actBaseRpyOut", m_actBaseRpy_),
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
  this->addOutPort("genTauOut", this->ports_.m_genTauOut_);
  this->addOutPort("actBasePoseOut", this->ports_.m_actBasePoseOut_);
  this->addOutPort("actBaseTformOut", this->ports_.m_actBaseTformOut_);
  this->addOutPort("actBasePosOut", this->ports_.m_actBasePosOut_);
  this->addOutPort("actBaseRpyOut", this->ports_.m_actBaseRpyOut_);
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
bool ActKinStabilizer::readInPortData(const double& dt, const GaitParam& gaitParam, const ActKinStabilizer::ControlMode& mode, ActKinStabilizer::Ports& ports, cnoid::BodyPtr refRobotRaw, cnoid::BodyPtr actRobotRaw, std::vector<GaitParam::Collision>& selfCollision){
  bool qRef_updated = false;
  if(ports.m_qRefIn_.isNew()){
    ports.m_qRefIn_.read();
    if(ports.m_qRef_.data.length() == refRobotRaw->numJoints()){
      for(int i=0;i<ports.m_qRef_.data.length();i++){
        if(std::isfinite(ports.m_qRef_.data[i])) {
          double q = ports.m_qRef_.data[i];
          double dq = (q - refRobotRaw->joint(i)->q()) / dt; // 指令関節角度はSequencePlayerなどによって滑らかになっていると仮定し、そのまま微分する. 初回のloopのときに値がとんでしまうが、MODE_IDLE状態なので問題ない
          double ddq = (dq - refRobotRaw->joint(i)->dq()) / dt; // 指令関節角度はSequencePlayerなどによって滑らかになっていると仮定し、そのまま微分する
          refRobotRaw->joint(i)->q() = q;
          refRobotRaw->joint(i)->dq() = dq;
          refRobotRaw->joint(i)->ddq() = ddq;
        }else{
          std::cerr << "m_qRef is not finite!" << std::endl;
        }
      }
      qRef_updated = true;
    }
  }
  if(ports.m_refTauIn_.isNew()){
    ports.m_refTauIn_.read();
    if(ports.m_refTau_.data.length() == refRobotRaw->numJoints()){
      for(int i=0;i<ports.m_refTau_.data.length();i++){
        if(std::isfinite(ports.m_refTau_.data[i])) refRobotRaw->joint(i)->u() = ports.m_refTau_.data[i];
        else std::cerr << "m_refTau is not finite!" << std::endl;
      }
    }
  }
  if(ports.m_refBasePosIn_.isNew()){
    ports.m_refBasePosIn_.read();
    if(std::isfinite(ports.m_refBasePos_.data.x) && std::isfinite(ports.m_refBasePos_.data.y) && std::isfinite(ports.m_refBasePos_.data.z)){
      cnoid::Vector3 p(ports.m_refBasePos_.data.x,ports.m_refBasePos_.data.y,ports.m_refBasePos_.data.z);
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
  if(ports.m_refBaseRpyIn_.isNew()){
    ports.m_refBaseRpyIn_.read();
    if(std::isfinite(ports.m_refBaseRpy_.data.r) && std::isfinite(ports.m_refBaseRpy_.data.p) && std::isfinite(ports.m_refBaseRpy_.data.y)){
      cnoid::Matrix3 R = cnoid::rotFromRpy(ports.m_refBaseRpy_.data.r, ports.m_refBaseRpy_.data.p, ports.m_refBaseRpy_.data.y);
      Eigen::AngleAxisd dR(R * refRobotRaw->rootLink()->R().transpose());  // reference frame.
      cnoid::Vector3 w = dR.angle() / dt * dR.axis(); // 指令ルート姿勢はSequencePlayerなどによって滑らかになっていると仮定し、そのまま微分する. 初回のloopのときに値がとんでしまうが、MODE_IDLE状態なので問題ない
      cnoid::Vector3 dw = (w - refRobotRaw->rootLink()->w()) / dt; // 指令ルート姿勢はSequencePlayerなどによって滑らかになっていると仮定し、そのまま微分する
      refRobotRaw->rootLink()->R() = R;
      refRobotRaw->rootLink()->w() = w;
      refRobotRaw->rootLink()->dw() = dw;
    } else {
      std::cerr << "m_refBaseRpy is not finite!" << std::endl;
    }
  }
  refRobotRaw->calcForwardKinematics();

  for(int i=0;i<ports.m_refEEWrenchIn_.size();i++){
    if(ports.m_refEEWrenchIn_[i]->isNew()){
      ports.m_refEEWrenchIn_[i]->read();
      if(ports.m_refEEWrench_[i].data.length() == 6){
        for(int j=0;j<6;j++){
          if(std::isfinite(ports.m_refEEWrench_[i].data[j])) refEEWrenchOrigin[i][j] = ports.m_refEEWrench_[i].data[j];
          else std::cerr << "m_refEEWrench is not finite!" << std::endl;
        }
      }
    }
  }

  for(int i=0;i<ports.m_refEEPoseIn_.size();i++){
    if(ports.m_refEEPoseIn_[i]->isNew()){
      ports.m_refEEPoseIn_[i]->read();
      if(std::isfinite(ports.m_refEEPose_[i].data.position.x) && std::isfinite(ports.m_refEEPose_[i].data.position.y) && std::isfinite(ports.m_refEEPose_[i].data.position.z) &&
         std::isfinite(ports.m_refEEPose_[i].data.orientation.r) && std::isfinite(ports.m_refEEPose_[i].data.orientation.p) && std::isfinite(ports.m_refEEPose_[i].data.orientation.y)){
        cnoid::Position pose;
        pose.translation()[0] = ports.m_refEEPose_[i].data.position.x;
        pose.translation()[1] = ports.m_refEEPose_[i].data.position.y;
        pose.translation()[2] = ports.m_refEEPose_[i].data.position.z;
        pose.linear() = cnoid::rotFromRpy(ports.m_refEEPose_[i].data.orientation.r, ports.m_refEEPose_[i].data.orientation.p, ports.m_refEEPose_[i].data.orientation.y);
        refEEPoseRaw[i].setGoal(pose, 0.3); // 0.3秒で補間
        ports.refEEPoseLastUpdateTime_ = ports.m_qRef_.tm;
      } else {
        std::cerr << "m_refEEPose is not finite!" << std::endl;
      }
    }
    refEEPoseRaw[i].interpolate(dt);
  }

  if(ports.m_qActIn_.isNew()){
    ports.m_qActIn_.read();
    if(ports.m_qAct_.data.length() == actRobotRaw->numJoints()){
      for(int i=0;i<ports.m_qAct_.data.length();i++){
        if(std::isfinite(ports.m_qAct_.data[i])) actRobotRaw->joint(i)->q() = ports.m_qAct_.data[i];
        else std::cerr << "m_qAct is not finite!" << std::endl;
      }
    }
  }
  if(ports.m_dqActIn_.isNew()){
    ports.m_dqActIn_.read();

    if(ports.m_dqAct_.data.length() == actRobotRaw->numJoints()){
      for(int i=0;i<ports.m_dqAct_.data.length();i++){
        if(std::isfinite(ports.m_dqAct_.data[i])) actRobotRaw->joint(i)->dq() = ports.m_dqAct_.data[i];
        else  std::cerr << "m_dqAct is not finite!" << std::endl;
      }
    }
  }
  if(ports.m_actImuIn_.isNew()){
    ports.m_actImuIn_.read();
    if(std::isfinite(ports.m_actImu_.data.r) && std::isfinite(ports.m_actImu_.data.p) && std::isfinite(ports.m_actImu_.data.y)){
      actRobotRaw->calcForwardKinematics();
      cnoid::RateGyroSensorPtr imu = actRobotRaw->findDevice<cnoid::RateGyroSensor>("gyrometer");
      cnoid::Matrix3 imuR = imu->link()->R() * imu->R_local();
      cnoid::Matrix3 actR = cnoid::rotFromRpy(ports.m_actImu_.data.r, ports.m_actImu_.data.p, ports.m_actImu_.data.y);
      actRobotRaw->rootLink()->R() = Eigen::Matrix3d(Eigen::AngleAxisd(actR) * Eigen::AngleAxisd(imuR.transpose() * actRobotRaw->rootLink()->R())); // 単純に3x3行列の空間でRを積算していると、だんだん数値誤差によって回転行列でなくなってしまう恐れがあるので念の為
    }else{
      std::cerr << "m_actImu is not finite!" << std::endl;
    }
  }
  actRobotRaw->calcForwardKinematics();

  cnoid::DeviceList<cnoid::ForceSensor> forceSensors(actRobotRaw->devices());
  for(int i=0;i<ports.m_actWrenchIn_.size();i++){
    if(ports.m_actWrenchIn_[i]->isNew()){
      ports.m_actWrenchIn_[i]->read();
      if(ports.m_actWrench_[i].data.length() == 6){
        for(int j=0;j<6;j++){
          if(std::isfinite(ports.m_actWrench_[i].data[j])) forceSensors[i]->F()[j] = ports.m_actWrench_[i].data[j];
          else std::cerr << "m_actWrench is not finite!" << std::endl;
        }
      }
    }
  }

  if(ports.m_selfCollisionIn_.isNew()) {
    ports.m_selfCollisionIn_.read();
    selfCollision.resize(ports.m_selfCollision_.data.length());
    for (int i=0; i<selfCollision.size(); i++){
      if(refRobotRaw->link(std::string(ports.m_selfCollision_.data[i].link1)) &&
         std::isfinite(ports.m_selfCollision_.data[i].point1.x) &&
         std::isfinite(ports.m_selfCollision_.data[i].point1.y) &&
         std::isfinite(ports.m_selfCollision_.data[i].point1.z) &&
         refRobotRaw->link(std::string(ports.m_selfCollision_.data[i].link2)) &&
         std::isfinite(ports.m_selfCollision_.data[i].point2.x) &&
         std::isfinite(ports.m_selfCollision_.data[i].point2.y) &&
         std::isfinite(ports.m_selfCollision_.data[i].point2.z) &&
         std::isfinite(ports.m_selfCollision_.data[i].direction21.x) &&
         std::isfinite(ports.m_selfCollision_.data[i].direction21.y) &&
         std::isfinite(ports.m_selfCollision_.data[i].direction21.z) &&
         std::isfinite(ports.m_selfCollision_.data[i].distance)){
        selfCollision[i].link1 = ports.m_selfCollision_.data[i].link1;
        selfCollision[i].point1[0] = ports.m_selfCollision_.data[i].point1.x;
        selfCollision[i].point1[1] = ports.m_selfCollision_.data[i].point1.y;
        selfCollision[i].point1[2] = ports.m_selfCollision_.data[i].point1.z;
        selfCollision[i].link2 = ports.m_selfCollision_.data[i].link2;
        selfCollision[i].point2[0] = ports.m_selfCollision_.data[i].point2.x;
        selfCollision[i].point2[1] = ports.m_selfCollision_.data[i].point2.y;
        selfCollision[i].point2[2] = ports.m_selfCollision_.data[i].point2.z;
        selfCollision[i].direction21[0] = ports.m_selfCollision_.data[i].direction21.x;
        selfCollision[i].direction21[1] = ports.m_selfCollision_.data[i].direction21.y;
        selfCollision[i].direction21[2] = ports.m_selfCollision_.data[i].direction21.z;
        selfCollision[i].distance = ports.m_selfCollision_.data[i].distance;
      }else{
        std::cerr << "m_selfCollision is not finite or has unknown link name!" << std::endl;
        selfCollision.resize(0);
        break;
      }
    }
  }

  if(ports.m_steppableRegionIn_.isNew()){
    ports.m_steppableRegionIn_.read();
    //steppableRegionを送るのは片足支持期のみ
    if (mode.isABCRunning() && // ABC起動中でないと現在支持脚という概念が無い
        ((gaitParam.footstepNodesList[0].isSupportPhase[RLEG] && !gaitParam.footstepNodesList[0].isSupportPhase[LLEG] && (ports.m_steppableRegion_.data.l_r == auto_stabilizer_msgs::RLEG)) ||
         (gaitParam.footstepNodesList[0].isSupportPhase[LLEG] && !gaitParam.footstepNodesList[0].isSupportPhase[RLEG] && (ports.m_steppableRegion_.data.l_r == auto_stabilizer_msgs::LLEG))) //現在支持脚と計算時支持脚が同じ
        ){
      int swingLeg = gaitParam.footstepNodesList[0].isSupportPhase[RLEG] ? LLEG : RLEG;
      int supportLeg = (swingLeg == RLEG) ? LLEG : RLEG;
      cnoid::Position supportPose = gaitParam.genCoords[supportLeg].value(); // TODO. 支持脚のgenCoordsとdstCoordsが異なることは想定していない
      cnoid::Position supportPoseHorizontal = mathutil::orientCoordToAxis(supportPose, cnoid::Vector3::UnitZ());
      steppableRegion.resize(ports.m_steppableRegion_.data.region.length());
      steppableHeight.resize(ports.m_steppableRegion_.data.region.length());
      for (int i=0; i<steppableRegion.size(); i++){
        double heightSum = 0.0;
        std::vector<cnoid::Vector3> vertices;
        for (int j=0; j<ports.m_steppableRegion_.data.region[i].length()/3; j++){
          if(!std::isfinite(ports.m_steppableRegion_.data.region[i][3*j]) || !std::isfinite(ports.m_steppableRegion_.data.region[i][3*j+1]) || !std::isfinite(ports.m_steppableRegion_.data.region[i][3*j+2])){
            std::cerr << "m_steppableRegion is not finite!" << std::endl;
            vertices.clear();
            break;
          }
          cnoid::Vector3 p = supportPoseHorizontal * cnoid::Vector3(ports.m_steppableRegion_.data.region[i][3*j],ports.m_steppableRegion_.data.region[i][3*j+1],ports.m_steppableRegion_.data.region[i][3*j+2]);
          heightSum += p[2];
          p[2] = 0.0;
          vertices.push_back(p);
        }
        double heightAverage = (ports.m_steppableRegion_.data.region[i].length()/3>0) ? heightSum / (ports.m_steppableRegion_.data.region[i].length()/3) : 0;
        steppableRegion[i] = mathutil::calcConvexHull(vertices);
        steppableHeight[i] = heightAverage;
      }
      ports.steppableRegionLastUpdateTime_ = ports.m_qRef_.tm;
    }
  }else{ //ports.m_steppableRegionIn_.isNew()
    if(std::abs(((long long)ports.steppableRegionLastUpdateTime_.sec - (long long)ports.m_qRef_.tm.sec) + 1e-9 * ((long long)ports.steppableRegionLastUpdateTime_.nsec - (long long)ports.m_qRef_.tm.nsec)) > 2.0){ // 2秒間steppableRegionが届いていない.  RTC::Timeはunsigned long型なので、符号付きの型に変換してから引き算
      steppableRegion.clear();
      steppableHeight.clear();
    }
  }

  if(ports.m_landingHeightIn_.isNew()) {
    ports.m_landingHeightIn_.read();
    if(std::isfinite(ports.m_landingHeight_.data.x) && std::isfinite(ports.m_landingHeight_.data.y) && std::isfinite(ports.m_landingHeight_.data.z) && std::isfinite(ports.m_landingHeight_.data.nx) && std::isfinite(ports.m_landingHeight_.data.ny) && std::isfinite(ports.m_landingHeight_.data.nz)){
      cnoid::Vector3 normal = cnoid::Vector3(ports.m_landingHeight_.data.nx, ports.m_landingHeight_.data.ny, ports.m_landingHeight_.data.nz);
      if(normal.norm() > 1.0 - 1e-2 && normal.norm() < 1.0 + 1e-2){ // ノルムがほぼ1
        if(mode.isABCRunning()){ // ABC起動中でないと現在支持脚という概念が無い
          if(ports.m_landingHeight_.data.l_r == auto_stabilizer_msgs::RLEG && gaitParam.footstepNodesList[0].isSupportPhase[RLEG] && !gaitParam.footstepNodesList[0].isSupportPhase[LLEG]) { //現在支持脚と計算時支持脚が同じ
            cnoid::Position supportPoseHorizontal = mathutil::orientCoordToAxis(gaitParam.genCoords[RLEG].value(), cnoid::Vector3::UnitZ());
            relLandingHeight = supportPoseHorizontal.translation()[2] + ports.m_landingHeight_.data.z;
            relLandingNormal = supportPoseHorizontal.linear() * normal.normalized();
          }else if(ports.m_landingHeight_.data.l_r == auto_stabilizer_msgs::LLEG && gaitParam.footstepNodesList[0].isSupportPhase[LLEG] && !gaitParam.footstepNodesList[0].isSupportPhase[RLEG]) { //現在支持脚と計算時支持脚が同じ
            cnoid::Position supportPoseHorizontal = mathutil::orientCoordToAxis(gaitParam.genCoords[LLEG].value(), cnoid::Vector3::UnitZ());
            relLandingHeight = supportPoseHorizontal.translation()[2] + ports.m_landingHeight_.data.z;
            relLandingNormal = supportPoseHorizontal.linear() * normal.normalized();
          }
        }
      }else{
        std::cerr << "m_landingHeight's norm != 1 !" << std::endl;
      }
    }else{
      std::cerr << "m_landingHeight is not finite!" << std::endl;
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

    cnoid::Vector3 basePos = basePose.translation();
    cnoid::Matrix3 baseR = basePose.linear();
    cnoid::Vector3 baseRpy = cnoid::rpyFromRot(basePose.linear());

    if(std::isfinite(basePos[0]) && std::isfinite(basePos[1]) && std::isfinite(basePos[2]) &&
       std::isfinite(baseRpy[0]) && std::isfinite(baseRpy[1]) && std::isfinite(baseRpy[2])){

      ports.m_actBasePose_.tm = ports.m_qRef_.tm;
      ports.m_actBasePose_.data.position.x = basePos[0];
      ports.m_actBasePose_.data.position.y = basePos[1];
      ports.m_actBasePose_.data.position.z = basePos[2];
      ports.m_actBasePose_.data.orientation.r = baseRpy[0];
      ports.m_actBasePose_.data.orientation.p = baseRpy[1];
      ports.m_actBasePose_.data.orientation.y = baseRpy[2];
      ports.m_actBasePoseOut_.write();

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
      ports.m_actBasePos_.data.x = basePos[0];
      ports.m_actBasePos_.data.y = basePos[1];
      ports.m_actBasePos_.data.z = basePos[2];
      ports.m_actBasePosOut_.write();
      ports.m_actBaseRpy_.tm = ports.m_qRef_.tm;
      ports.m_actBaseRpy_.data.r = baseRpy[0];
      ports.m_actBaseRpy_.data.p = baseRpy[1];
      ports.m_actBaseRpy_.data.y = baseRpy[2];
      ports.m_actBaseRpyOut_.write();
    }else{
      std::cerr << "m_actBasePose is not finite!" << std::endl;
    }
  }


  // only for logger. (IDLE時の出力や、モード遷移時の連続性はてきとうで良い)
  if(mode.isABCRunning()){
    ports.m_actCog_.tm = ports.m_qRef_.tm;
    ports.m_actCog_.data.x = gaitParam.robot->body->centerOfMass()[0];
    ports.m_actCog_.data.y = gaitParam.robot->body->centerOfMass()[1];
    ports.m_actCog_.data.z = gaitParam.robot->body->centerOfMass()[2];
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


  if(!ActKinStabilizer::readInPortData(dt, this->gaitParam_, this->mode_, this->ports_, this->gaitParam_.refRobotRaw, this->gaitParam_.actRobotRaw, this->gaitParam_.refEEWrenchOrigin, this->gaitParam_.refEEPoseRaw, this->gaitParam_.selfCollision, this->gaitParam_.steppableRegion, this->gaitParam_.steppableHeight, this->gaitParam_.relLandingHeight, this->gaitParam_.relLandingNormal)) return RTC::RTC_OK;  // qRef が届かなければ何もしない


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

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
    for(int i=0;i<this->gaitParam_.genRobot->numJoints();i++){
      cnoid::LinkPtr joint = this->gaitParam_.genRobot->joint(i);
      double climit = 0.0, gearRatio = 0.0, torqueConst = 0.0;
      joint->info()->read("climit",climit); joint->info()->read("gearRatio",gearRatio); joint->info()->read("torqueConst",torqueConst);
      this->gaitParam_.maxTorque[i] = std::max(climit * gearRatio * torqueConst, 0.0);
    }
    std::string jointLimitTableStr; this->getProperty("joint_limit_table",jointLimitTableStr);
    {
      std::vector<std::shared_ptr<joint_limit_table::JointLimitTable> > jointLimitTables = joint_limit_table::readJointLimitTablesFromProperty (this->gaitParam_.actRobot, jointLimitTableStr);
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
    for(int i=0;i<this->gaitParam_.actRobot->numJoints();i++){
      cnoid::LinkPtr joint = this->gaitParam_.actRobot->joint(i);
      if(joint->q_upper() - joint->q_lower() > 0.002){
        joint->setJointRange(joint->q_lower()+0.001,joint->q_upper()-0.001);
      }
      // JointVelocityについて. 1.0だと安全.4.0は脚.10.0はlapid manipulation らしい. limitを小さくしすぎた状態で、速い指令を送ると、狭いlimitの中で高優先度タスクを頑張って満たそうとすることで、低優先度タスクを満たす余裕がなくエラーが大きくなってしまうことに注意.
      if(joint->dq_upper() - joint->dq_lower() > 0.02){
        joint->setJointVelocityRange(joint->dq_lower()+0.01,joint->dq_upper()-0.01);
      }
    }
  }


  // init Stabilizer
  this->stabilizer_.init(this->gaitParam_, this->gaitParam_.actRobotTqc, this->gaitParam_.actRobot);

  // initialize parameters
  this->loop_ = 0;

  return RTC::RTC_OK;
}

// static function
bool ActKinStabilizer::readInPortData(const double& dt, const GaitParam& gaitParam, const ActKinStabilizer::ControlMode& mode, ActKinStabilizer::Ports& ports, cnoid::BodyPtr refRobotRaw, cnoid::BodyPtr actRobotRaw, std::vector<cnoid::Vector6>& refEEWrenchOrigin, std::vector<cpp_filters::TwoPointInterpolatorSE3>& refEEPoseRaw, std::vector<GaitParam::Collision>& selfCollision, std::vector<std::vector<cnoid::Vector3> >& steppableRegion, std::vector<double>& steppableHeight, double& relLandingHeight, cnoid::Vector3& relLandingNormal){
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
bool ActKinStabilizer::execActKinStabilizer(const ActKinStabilizer::ControlMode& mode, GaitParam& gaitParam, double dt, const FootStepGenerator& footStepGenerator, const LegCoordsGenerator& legCoordsGenerator, const RefToGenFrameConverter& refToGenFrameConverter, const ActToGenFrameConverter& actToGenFrameConverter, const ImpedanceController& impedanceController, const Stabilizer& stabilizer, const ExternalForceHandler& externalForceHandler, const LegManualController& legManualController, const CmdVelGenerator& cmdVelGenerator) {
  if(mode.isSyncToABCInit()){ // startAutoBalancer直後の初回. gaitParamのリセット
    refToGenFrameConverter.initGenRobot(gaitParam,
                                        gaitParam.genRobot, gaitParam.footMidCoords, gaitParam.genCog, gaitParam.genCogVel, gaitParam.genCogAcc);
    actToGenFrameConverter.initOutput(gaitParam,
                                      gaitParam.actCogVel, gaitParam.actRootVel);
    externalForceHandler.initExternalForceHandlerOutput(gaitParam,
                                                        gaitParam.omega, gaitParam.l);
    impedanceController.initImpedanceOutput(gaitParam,
                                            gaitParam.icEEOffset);
    footStepGenerator.initFootStepNodesList(gaitParam,
                                            gaitParam.footstepNodesList, gaitParam.srcCoords, gaitParam.dstCoordsOrg, gaitParam.remainTimeOrg, gaitParam.swingState, gaitParam.elapsedTime, gaitParam.prevSupportPhase);
    legCoordsGenerator.initLegCoords(gaitParam,
                                     gaitParam.refZmpTraj, gaitParam.genCoords);
  }

  // FootOrigin座標系を用いてrefRobotRawをgenerate frameに投影しrefRobotとする
  refToGenFrameConverter.convertFrame(gaitParam, dt,
                                      gaitParam.refRobot, gaitParam.refEEPose, gaitParam.refEEWrench, gaitParam.refdz, gaitParam.footMidCoords);

  // FootOrigin座標系を用いてactRobotRawをgenerate frameに投影しactRobotとする
  actToGenFrameConverter.convertFrame(gaitParam, dt,
                                      gaitParam.actRobot, gaitParam.actEEPose, gaitParam.actEEWrench, gaitParam.actCog, gaitParam.actCogVel, gaitParam.actRootVel);

  // 目標外力に応じてオフセットを計算する
  externalForceHandler.handleExternalForce(gaitParam, mode.isSTRunning(), dt,
                                           gaitParam.omega, gaitParam.l);

  // Impedance Controller
  impedanceController.calcImpedanceControl(dt, gaitParam,
                                           gaitParam.icEEOffset, gaitParam.icEETargetPose);

  // Manual Control Modeの足の現在位置をreferenceで上書きする
  legManualController.legManualControl(gaitParam, dt,
                                       gaitParam.genCoords, gaitParam.footstepNodesList, gaitParam.isManualControlMode);

  // CmdVelGenerator
  cmdVelGenerator.calcCmdVel(gaitParam,
                             gaitParam.cmdVel);

  // ActKinBalancer
  footStepGenerator.procFootStepNodesList(gaitParam, dt, mode.isSTRunning(),
                                          gaitParam.footstepNodesList, gaitParam.srcCoords, gaitParam.dstCoordsOrg, gaitParam.remainTimeOrg, gaitParam.swingState, gaitParam.elapsedTime, gaitParam.prevSupportPhase, gaitParam.relLandingHeight);
  footStepGenerator.calcFootSteps(gaitParam, dt, mode.isSTRunning(),
                                  gaitParam.debugData, //for log
                                  gaitParam.footstepNodesList);
  legCoordsGenerator.calcLegCoords(gaitParam, dt, mode.isSTRunning(),
                                   gaitParam.refZmpTraj, gaitParam.genCoords, gaitParam.swingState);
  legCoordsGenerator.calcEETargetPose(gaitParam, dt,
                                      gaitParam.abcEETargetPose, gaitParam.abcEETargetVel, gaitParam.abcEETargetAcc);

  // Stabilizer
  stabilizer.execStabilizer(gaitParam, dt, mode.isSTRunning(),
                            gaitParam.debugData, //for log
                            gaitParam.actRobotTqc, gaitParam.genRobot, gaitParam.genCog, gaitParam.genCogVel, gaitParam.genCogAcc);

  return true;
}

// static function
bool ActKinStabilizer::writeOutPortData(ActKinStabilizer::Ports& ports, const ActKinStabilizer::ControlMode& mode, double dt, const GaitParam& gaitParam, cpp_filters::TwoPointInterpolatorSE3& outputRootPoseFilter, std::vector<cpp_filters::TwoPointInterpolator<double> >& outputJointAngleFilter){
  {
    // q
    ports.m_q_.tm = ports.m_qRef_.tm;
    ports.m_q_.data.length(gaitParam.genRobot->numJoints());
    for(int i=0;i<gaitParam.genRobot->numJoints();i++){
      if(mode.now() == ActKinStabilizer::ControlMode::MODE_IDLE || mode.now() == ActKinStabilizer::ControlMode::MODE_SYNC_TO_ABC || mode.now() == ActKinStabilizer::ControlMode::MODE_ABC || mode.now() == ActKinStabilizer::ControlMode::MODE_SYNC_TO_IDLE || !gaitParam.jointControllable[i]){
        outputJointAngleFilter[i].reset(gaitParam.refRobotRaw->joint(i)->q()); // shの値をそのまま出力
      }else if(mode.now() == ActKinStabilizer::ControlMode::MODE_SYNC_TO_EMG || mode.now() == ActKinStabilizer::ControlMode::MODE_EMG){
        // 今の値のまま
      }else if(mode.now() == ActKinStabilizer::ControlMode::MODE_SYNC_TO_RELEASE_EMG){
        outputJointAngleFilter[i].setGoal(gaitParam.refRobotRaw->joint(i)->q(), mode.remainTime());
      }else if(mode.now() == ActKinStabilizer::ControlMode::MODE_SYNC_TO_ST || mode.now() == ActKinStabilizer::ControlMode::MODE_ST){
        outputJointAngleFilter[i].setGoal(gaitParam.actRobotRaw->joint(i)->q(), 0.3); // 0.3秒で遅れてactualで上書き
      }

      outputJointAngleFilter[i].interpolate(dt);
      if(std::isfinite(outputJointAngleFilter[i].value())) ports.m_q_.data[i] = outputJointAngleFilter[i].value();
      else std::cerr << "m_q is not finite!" << std::endl;
    }
    ports.m_qOut_.write();
  }

  {
    // tau
    ports.m_genTau_.tm = ports.m_qRef_.tm;
    ports.m_genTau_.data.length(gaitParam.actRobotTqc->numJoints());
    for(int i=0;i<gaitParam.actRobotTqc->numJoints();i++){
      double value = 0.0;
      if(mode.now() == ActKinStabilizer::ControlMode::MODE_IDLE || mode.now() == ActKinStabilizer::ControlMode::MODE_SYNC_TO_ABC || mode.now() == ActKinStabilizer::ControlMode::MODE_ABC || mode.now() == ActKinStabilizer::ControlMode::MODE_SYNC_TO_IDLE || !gaitParam.jointControllable[i]){
        value = gaitParam.refRobotRaw->joint(i)->u();
      }else if(mode.now() == ActKinStabilizer::ControlMode::MODE_SYNC_TO_EMG || mode.now() == ActKinStabilizer::ControlMode::MODE_EMG){
        value = 0.0; // stopST補間中は、tauを少しずつ0に減らして行きたくなるが、吹っ飛ぶことがあるので一気に0にした方が安全
      }else if(mode.now() == ActKinStabilizer::ControlMode::MODE_SYNC_TO_RELEASE_EMG){
        value = mode.transitionRatio() * gaitParam.refRobotRaw->joint(i)->u();
      }else if(mode.now() == ActKinStabilizer::ControlMode::MODE_SYNC_TO_ST || mode.now() == ActKinStabilizer::ControlMode::MODE_ST){
        value = gaitParam.actRobotTqc->joint(i)->u();
      }

      if(std::isfinite(value)) ports.m_genTau_.data[i] = value;
      else std::cerr << "m_genTau is not finite!" << std::endl;
    }
    ports.m_genTauOut_.write();
  }

  // Gains
  if(!CORBA::is_nil(ports.m_robotHardwareService0_._ptr()) && //コンシューマにプロバイダのオブジェクト参照がセットされていない(接続されていない)状態
     !ports.m_robotHardwareService0_->_non_existent()){ //プロバイダのオブジェクト参照は割り当てられているが、相手のオブジェクトが非活性化 (RTC は Inactive 状態) になっている状態
    for(int i=0;i<gaitParam.genRobot->numJoints();i++){
      // Stabilizerが動いている間にonDeactivated()->onActivated()が呼ばれると、ゲインがもとに戻らない. onDeactivated()->onActivated()が呼ばれるのはサーボオン直前で、通常、サーボオン時にゲインを指令するので、問題ない.
      if(!gaitParam.jointControllable[i]){
        // pass
      }else if(mode.now() == ActKinStabilizer::ControlMode::MODE_SYNC_TO_ST && mode.pre() != mode.now()){
        ports.m_robotHardwareService0_->setServoPGainPercentageWithTime(gaitParam.actRobotTqc->joint(i)->name().c_str(),0.0*100.0,mode.remainTime());
        ports.m_robotHardwareService0_->setServoDGainPercentageWithTime(gaitParam.actRobotTqc->joint(i)->name().c_str(),0.0*100.0,mode.remainTime());
      }else if(mode.now() == ActKinStabilizer::ControlMode::MODE_SYNC_TO_EMG && mode.pre() != mode.now()){
        ports.m_robotHardwareService0_->setServoPGainPercentageWithTime(gaitParam.actRobotTqc->joint(i)->name().c_str(),100.0*100.0,mode.remainTime());
        ports.m_robotHardwareService0_->setServoDGainPercentageWithTime(gaitParam.actRobotTqc->joint(i)->name().c_str(),100.0*100.0,mode.remainTime());
      }else{
        // pass
      }
    }
  }

  {
    // basePose (hrpsys_odom)
    if(mode.now() == ActKinStabilizer::ControlMode::MODE_IDLE){
      outputRootPoseFilter.reset(gaitParam.actRobotRaw->rootLink()->T()); // pはref値. RはIMUが入っている
    }else if(mode.now() == ActKinStabilizer::ControlMode::MODE_SYNC_TO_IDLE){
      outputRootPoseFilter.setGoal(gaitParam.actRobotRaw->rootLink()->T(), mode.remainTime()); // pはref値. RはIMUが入っている
    }else{
      outputRootPoseFilter.reset(gaitParam.actRobot->rootLink()->T()); // pはref値. RはIMUが入っている
    }
    outputRootPoseFilter.interpolate(dt);
    cnoid::Position basePose = outputRootPoseFilter.value();

    cnoid::Vector3 basePos = basePose.translation();
    cnoid::Matrix3 baseR = basePose.linear();
    cnoid::Vector3 baseRpy = cnoid::rpyFromRot(basePose.linear());

    if(std::isfinite(basePos[0]) && std::isfinite(basePos[1]) && std::isfinite(basePos[2]) &&
       std::isfinite(baseRpy[0]) && std::isfinite(baseRpy[1]) && std::isfinite(baseRpy[2])){

      ports.m_genBasePose_.tm = ports.m_qRef_.tm;
      ports.m_genBasePose_.data.position.x = basePos[0];
      ports.m_genBasePose_.data.position.y = basePos[1];
      ports.m_genBasePose_.data.position.z = basePos[2];
      ports.m_genBasePose_.data.orientation.r = baseRpy[0];
      ports.m_genBasePose_.data.orientation.p = baseRpy[1];
      ports.m_genBasePose_.data.orientation.y = baseRpy[2];
      ports.m_genBasePoseOut_.write();

      ports.m_genBaseTform_.tm = ports.m_qRef_.tm;
      ports.m_genBaseTform_.data.length(12);
      for(int i=0;i<3;i++){
        ports.m_genBaseTform_.data[i] = basePos[i];
      }
      for(int i=0;i<3;i++){
        for(int j=0;j<3;j++){
          ports.m_genBaseTform_.data[3+i*3+j] = baseR(i,j);// row major
        }
      }
      ports.m_genBaseTformOut_.write();

      ports.m_genBasePos_.tm = ports.m_qRef_.tm;
      ports.m_genBasePos_.data.x = basePos[0];
      ports.m_genBasePos_.data.y = basePos[1];
      ports.m_genBasePos_.data.z = basePos[2];
      ports.m_genBasePosOut_.write();
      ports.m_genBaseRpy_.tm = ports.m_qRef_.tm;
      ports.m_genBaseRpy_.data.r = baseRpy[0];
      ports.m_genBaseRpy_.data.p = baseRpy[1];
      ports.m_genBaseRpy_.data.y = baseRpy[2];
      ports.m_genBaseRpyOut_.write();
    }else{
      std::cerr << "m_genBasePose is not finite!" << std::endl;
    }
  }

  // acc ref
  {
    cnoid::Vector3 genImuAcc = cnoid::Vector3::Zero(); // imu frame
    if(mode.isABCRunning()){
      cnoid::RateGyroSensorPtr imu = gaitParam.actRobot->findDevice<cnoid::RateGyroSensor>("gyrometer"); // actrobot imu
      cnoid::Matrix3 imuR = imu->link()->R() * imu->R_local(); // generate frame
      genImuAcc/*imu frame*/ = imuR.transpose() * gaitParam.genCogAcc/*generate frame*/; // 本当は重心の加速ではなく、関節の加速等を考慮したimuセンサの加速を直接与えたいが、関節角度ベースのinverse-kinematicsを使う以上モデルのimuセンサの位置の加速が不連続なものになることは避けられないので、重心の加速を用いている. この出力の主な用途は歩行時の姿勢推定のため、重心の加速が考慮できればだいたい十分.
    }
    if(std::isfinite(genImuAcc[0]) && std::isfinite(genImuAcc[1]) && std::isfinite(genImuAcc[2])){
      ports.m_genImuAcc_.tm = ports.m_qRef_.tm;
      ports.m_genImuAcc_.data.ax = genImuAcc[0];
      ports.m_genImuAcc_.data.ay = genImuAcc[1];
      ports.m_genImuAcc_.data.az = genImuAcc[2];
      ports.m_genImuAccOut_.write();
    }else{
      std::cerr << "m_genImuAcc is not finite!" << std::endl;
    }
  }

  //landngTarget
  if(mode.isABCRunning() && // ABC起動中でないと支持脚という概念が無い
     gaitParam.footstepNodesList.size() >= 2 &&
     ((gaitParam.footstepNodesList[0].isSupportPhase[RLEG] && !gaitParam.footstepNodesList[0].isSupportPhase[LLEG]) ||
      (!gaitParam.footstepNodesList[0].isSupportPhase[RLEG] && gaitParam.footstepNodesList[0].isSupportPhase[LLEG])) && // 今が片足支持
     (gaitParam.footstepNodesList[1].isSupportPhase[RLEG] && gaitParam.footstepNodesList[1].isSupportPhase[LLEG]) // 次が両足支持
     ) {
    int supportLeg = gaitParam.footstepNodesList[0].isSupportPhase[RLEG] ? RLEG : LLEG;
    int swingLeg = gaitParam.footstepNodesList[0].isSupportPhase[RLEG] ? LLEG : RLEG;
    ports.m_landingTarget_.tm = ports.m_qRef_.tm;
    cnoid::Position supportPoseHorizontal = mathutil::orientCoordToAxis(gaitParam.genCoords[supportLeg].value(), cnoid::Vector3::UnitZ());
    ports.m_landingTarget_.data.x = (supportPoseHorizontal.inverse() * gaitParam.footstepNodesList[0].dstCoords[swingLeg].translation())[0];
    ports.m_landingTarget_.data.y = (supportPoseHorizontal.inverse() * gaitParam.footstepNodesList[0].dstCoords[swingLeg].translation())[1];
    ports.m_landingTarget_.data.z = (supportPoseHorizontal.inverse() * gaitParam.footstepNodesList[0].dstCoords[swingLeg].translation())[2];
    ports.m_landingTarget_.data.l_r = gaitParam.footstepNodesList[0].isSupportPhase[RLEG] ? auto_stabilizer_msgs::RLEG : auto_stabilizer_msgs::LLEG;
    ports.m_landingTargetOut_.write();
  }

  // actEEPose actEEWrench (for wholebodymasterslave)
  if(mode.isABCRunning()){
    for(int i=0;i<gaitParam.endEffectors.size();i++){
      ports.m_actEEPose_[i].tm = ports.m_qRef_.tm;
      ports.m_actEEPose_[i].data.position.x = gaitParam.actEEPose[i].translation()[0];
      ports.m_actEEPose_[i].data.position.y = gaitParam.actEEPose[i].translation()[1];
      ports.m_actEEPose_[i].data.position.z = gaitParam.actEEPose[i].translation()[2];
      cnoid::Vector3 rpy = cnoid::rpyFromRot(gaitParam.actEEPose[i].linear());
      ports.m_actEEPose_[i].data.orientation.r = rpy[0];
      ports.m_actEEPose_[i].data.orientation.p = rpy[1];
      ports.m_actEEPose_[i].data.orientation.y = rpy[2];
      ports.m_actEEPoseOut_[i]->write();
    }
    for(int i=0;i<gaitParam.endEffectors.size();i++){
      ports.m_actEEWrench_[i].tm = ports.m_qRef_.tm;
      ports.m_actEEWrench_[i].data.length(6);
      for(int j=0;j<6;j++) ports.m_actEEWrench_[i].data[j] = gaitParam.actEEWrench[i][j];
      ports.m_actEEWrenchOut_[i]->write();
    }
  }

  // only for logger. (IDLE時の出力や、モード遷移時の連続性はてきとうで良い)
  if(mode.isABCRunning()){
    ports.m_genCog_.tm = ports.m_qRef_.tm;
    ports.m_genCog_.data.x = gaitParam.genCog[0];
    ports.m_genCog_.data.y = gaitParam.genCog[1];
    ports.m_genCog_.data.z = gaitParam.genCog[2];
    ports.m_genCogOut_.write();
    cnoid::Vector3 genDcm = gaitParam.genCog + gaitParam.genCogVel / gaitParam.omega;
    ports.m_genDcm_.tm = ports.m_qRef_.tm;
    ports.m_genDcm_.data.x = genDcm[0];
    ports.m_genDcm_.data.y = genDcm[1];
    ports.m_genDcm_.data.z = genDcm[2];
    ports.m_genDcmOut_.write();
    ports.m_genZmp_.tm = ports.m_qRef_.tm;
    ports.m_genZmp_.data.x = gaitParam.refZmpTraj[0].getStart()[0];
    ports.m_genZmp_.data.y = gaitParam.refZmpTraj[0].getStart()[1];
    ports.m_genZmp_.data.z = gaitParam.refZmpTraj[0].getStart()[2];
    ports.m_genZmpOut_.write();
    ports.m_tgtZmp_.tm = ports.m_qRef_.tm;
    ports.m_tgtZmp_.data.x = gaitParam.debugData.stTargetZmp[0];
    ports.m_tgtZmp_.data.y = gaitParam.debugData.stTargetZmp[1];
    ports.m_tgtZmp_.data.z = gaitParam.debugData.stTargetZmp[2];
    ports.m_tgtZmpOut_.write();
    ports.m_actCog_.tm = ports.m_qRef_.tm;
    ports.m_actCog_.data.x = gaitParam.actCog[0];
    ports.m_actCog_.data.y = gaitParam.actCog[1];
    ports.m_actCog_.data.z = gaitParam.actCog[2];
    ports.m_actCogOut_.write();
    cnoid::Vector3 actDcm = gaitParam.actCog + gaitParam.actCogVel.value() / gaitParam.omega;
    ports.m_actDcm_.tm = ports.m_qRef_.tm;
    ports.m_actDcm_.data.x = actDcm[0];
    ports.m_actDcm_.data.y = actDcm[1];
    ports.m_actDcm_.data.z = actDcm[2];
    ports.m_actDcmOut_.write();
    ports.m_dstLandingPos_.tm = ports.m_qRef_.tm;
    ports.m_dstLandingPos_.data.length(6);
    ports.m_dstLandingPos_.data[0] = gaitParam.footstepNodesList[0].dstCoords[RLEG].translation()[0];
    ports.m_dstLandingPos_.data[1] = gaitParam.footstepNodesList[0].dstCoords[RLEG].translation()[1];
    ports.m_dstLandingPos_.data[2] = gaitParam.footstepNodesList[0].dstCoords[RLEG].translation()[2];
    ports.m_dstLandingPos_.data[3] = gaitParam.footstepNodesList[0].dstCoords[LLEG].translation()[0];
    ports.m_dstLandingPos_.data[4] = gaitParam.footstepNodesList[0].dstCoords[LLEG].translation()[1];
    ports.m_dstLandingPos_.data[5] = gaitParam.footstepNodesList[0].dstCoords[LLEG].translation()[2];
    ports.m_dstLandingPosOut_.write();
    ports.m_remainTime_.tm = ports.m_qRef_.tm;
    ports.m_remainTime_.data.length(1);
    ports.m_remainTime_.data[0] = gaitParam.footstepNodesList[0].remainTime;
    ports.m_remainTimeOut_.write();
    ports.m_genCoords_.tm = ports.m_qRef_.tm;
    ports.m_genCoords_.data.length(12);
    for (int i=0; i<3; i++) {
      ports.m_genCoords_.data[0+i] = gaitParam.genCoords[RLEG].value().translation()[i];
      ports.m_genCoords_.data[3+i] = gaitParam.genCoords[LLEG].value().translation()[i];
      ports.m_genCoords_.data[6+i] = gaitParam.genCoords[RLEG].getGoal().translation()[i];
      ports.m_genCoords_.data[9+i] = gaitParam.genCoords[LLEG].getGoal().translation()[i];
    }
    ports.m_genCoordsOut_.write();
    {
      ports.m_captureRegion_.tm = ports.m_qRef_.tm;
      int sum = 0;
      for (int i=0; i<gaitParam.debugData.capturableHulls.size(); i++) sum+=gaitParam.debugData.capturableHulls[i].size();
      ports.m_captureRegion_.data.length(sum*2);
      int index = 0;
      for (int i=0; i<gaitParam.debugData.capturableHulls.size(); i++) {
        for (int j=0; j<gaitParam.debugData.capturableHulls[i].size(); j++) {
          ports.m_captureRegion_.data[index+0] = gaitParam.debugData.capturableHulls[i][j][0];
          ports.m_captureRegion_.data[index+1] = gaitParam.debugData.capturableHulls[i][j][1];
          index+=2;
        }
      }
      ports.m_captureRegionOut_.write();
    }
    {
      ports.m_steppableRegionLog_.tm = ports.m_qRef_.tm;
      ports.m_steppableRegionNumLog_.tm = ports.m_qRef_.tm;
      int sum = 0;
      for (int i=0; i<gaitParam.steppableRegion.size(); i++) sum+=gaitParam.steppableRegion[i].size();
      ports.m_steppableRegionLog_.data.length(sum*2);
      ports.m_steppableRegionNumLog_.data.length(gaitParam.steppableRegion.size());
      int index=0;
      for (int i=0; i<gaitParam.steppableRegion.size(); i++) {
        for (int j=0; j<gaitParam.steppableRegion[i].size(); j++) {
          ports.m_steppableRegionLog_.data[index+0] = gaitParam.steppableRegion[i][j][0];
          ports.m_steppableRegionLog_.data[index+1] = gaitParam.steppableRegion[i][j][1];
          index+=2;
        }
        ports.m_steppableRegionNumLog_.data[i] = gaitParam.steppableRegion[i].size();
      }
      ports.m_steppableRegionLogOut_.write();
      ports.m_steppableRegionNumLogOut_.write();
    }
    ports.m_strideLimitationHull_.tm = ports.m_qRef_.tm;
    ports.m_strideLimitationHull_.data.length(gaitParam.debugData.strideLimitationHull.size()*2);
    for (int i=0; i<gaitParam.debugData.strideLimitationHull.size(); i++) {
      ports.m_strideLimitationHull_.data[i*2+0] = gaitParam.debugData.strideLimitationHull[i][0];
      ports.m_strideLimitationHull_.data[i*2+1] = gaitParam.debugData.strideLimitationHull[i][1];
    }
    ports.m_strideLimitationHullOut_.write();
    ports.m_cpViewerLog_.tm = ports.m_qRef_.tm;
    ports.m_cpViewerLog_.data.length(gaitParam.debugData.cpViewerLog.size());
    for (int i=0; i<gaitParam.debugData.cpViewerLog.size(); i++) {
      ports.m_cpViewerLog_.data[i] = gaitParam.debugData.cpViewerLog[i];
    }
    ports.m_cpViewerLogOut_.write();
    for(int i=0;i<gaitParam.endEffectors.size();i++){
      ports.m_tgtEEWrench_[i].tm = ports.m_qRef_.tm;
      ports.m_tgtEEWrench_[i].data.length(6);
      for(int j=0;j<6;j++) ports.m_tgtEEWrench_[i].data[j] = gaitParam.debugData.stEETargetWrench[i][j];
      ports.m_tgtEEWrenchOut_[i]->write();
    }
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

  if(!ActKinStabilizer::readInPortData(this->dt_, this->gaitParam_, this->mode_, this->ports_, this->gaitParam_.refRobotRaw, this->gaitParam_.actRobotRaw, this->gaitParam_.refEEWrenchOrigin, this->gaitParam_.refEEPoseRaw, this->gaitParam_.selfCollision, this->gaitParam_.steppableRegion, this->gaitParam_.steppableHeight, this->gaitParam_.relLandingHeight, this->gaitParam_.relLandingNormal)) return RTC::RTC_OK;  // qRef が届かなければ何もしない

  // 内部補間器を進める
  this->mode_.update(this->dt_);
  this->gaitParam_.update(this->dt_);
  this->refToGenFrameConverter_.update(this->dt_);
  this->stabilizer_.update(this->dt_);

  if(this->mode_.isSTRunning()) {
    if(this->mode_.isSyncToSTInit()){ // startStabilizer直後の初回. 内部パラメータのリセット
      this->gaitParam_.reset();
      this->refToGenFrameConverter_.reset();
      this->actToGenFrameConverter_.reset();
      this->externalForceHandler_.reset();
      this->footStepGenerator_.reset();
      this->impedanceController_.reset();
      this->legCoordsGenerator_.reset();
      this->stabilizer_.reset();
    }
    ActKinStabilizer::execActKinStabilizer(this->mode_, this->gaitParam_, this->dt_, this->footStepGenerator_, this->legCoordsGenerator_, this->refToGenFrameConverter_, this->actToGenFrameConverter_, this->impedanceController_, this->stabilizer_,this->externalForceHandler_, this->legManualController_, this->cmdVelGenerator_);
  }

  ActKinStabilizer::writeOutPortData(this->ports_, this->mode_, this->dt_, this->gaitParam_, this->outputRootPoseFilter_, this->outputJointAngleFilter_);

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

bool ActKinStabilizer::setFootSteps(const actkin_stabilizer::ActKinStabilizerService::FootstepSequence& fs){
  actkin_stabilizer::ActKinStabilizerService::StepParamSequence sps;
  sps.length(fs.length());
  for(int i=0;i<fs.length();i++){
    sps[i].step_height = this->footStepGenerator_.defaultStepHeight;
    sps[i].step_time = this->footStepGenerator_.defaultStepTime;
    sps[i].swing_end = false;
  }
  return this->setFootStepsWithParam(fs, sps); // この中でmutexをとるので、setFootSteps関数ではmutexはとらない
}


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

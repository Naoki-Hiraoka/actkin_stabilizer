#include "State.h"
#include <unordered_set>
#include <eigen_rtm_conversions/eigen_rtm_conversions.h>
#include <rtm_data_tools/rtm_data_tools.h>
#include <cnoid/src/Body/InverseDynamics.h>
#include <cnoid/Jacobian>

namespace actkin_stabilizer {
  void State::init(const cnoid::BodyPtr& robot_, const std::vector<std::vector<std::shared_ptr<joint_limit_table::JointLimitTable> > >& jointLimitTables_){

    this->robot = robot_;
    for(int i=0;i<this->robot->numJoints();i++){
      cnoid::LinkPtr joint = this->robot->joint(i);
      double climit = 0.0, gearRatio = 0.0, torqueConst = 0.0;
      joint->info()->read("climit",climit); joint->info()->read("gearRatio",gearRatio); joint->info()->read("torqueConst",torqueConst);
      double maxTorque =  std::max(climit * gearRatio * torqueConst, 0.0);
      if(maxTorque > 0.0) joint->setJointEffortRange(-maxTorque,maxTorque);
    }
    this->softMaxTorque.resize(this->robot->numJoints(),std::numeric_limits<double>::max());
    this->jointControllable.resize(this->robot->numJoints(),true);

    if(jointLimitTables_.size() != this->robot->numJoints()) {
      std::cerr << "\x1b[31m[" << __FUNCTION__ << "jointLimitTables_.size() != this->robot->numJoints()" << "\x1b[39m" << std::endl;
      this->jointLimitTables.resize(this->robot->numJoints());
    }else{
      this->jointLimitTables = jointLimitTables_;
    }

    // constraints
    this->jointLimitConstraints.resize(this->robot->numJoints());
    for(int i=0;i<this->robot->numJoints();i++){
      std::shared_ptr<aik_constraint_joint_limit_table::JointLimitMinMaxTableConstraint> constraint = std::make_shared<aik_constraint_joint_limit_table::JointLimitMinMaxTableConstraint>();
      constraint->joint() = this->robot->joint(i);
      constraint->jointLimitTables() = this->jointLimitTables[i];
      // 動歩行時の股関節や足首では、関節角度上下限直前で急減速する動作がある. これをlimit内判定するために、pgain/dgainを大きくせよ. 
      constraint->pgain() = 800;
      constraint->dgain() = 60;
      constraint->maxAcc() = 1e5; // 常に満たす不等式制約なので1e5でよい. 逆に常にチェックしてくれないと困る
      constraint->maxAccByPosError() = 1e5;
      constraint->maxAccByVelError() = 1e5;
      constraint->weight() = 1.0;
      this->jointLimitConstraints[i] = constraint;
    }
    this->eomConstraint = std::make_shared<aik_constraint::EOMConstraint>();
    this->eomConstraint->robot() = robot;

    this->linkNameMap[std::string("")] = nullptr; //world
    for(int l=0;l<this->robot->numLinks() ; l++){
      cnoid::SgGroup* shape = this->robot->link(l)->shape();
      if(shape && shape->numChildObjects() > 0 && shape->child(0)->name().size()!=0){
        this->linkNameMap[shape->child(0)->name()] = this->robot->link(l);
      }
    }

    return;
  };

  void State::onStartStabilizer(){
    this->cogVel.reset(cnoid::Vector3::Zero());
    return;
  }

  void State::updateRobotFromIdl(const RTC::TimedDoubleSeq& m_qAct, const RTC::TimedDoubleSeq& m_dqAct, const RTC::TimedPose3D& m_actBasePose, const RTC::TimedVelocity3D& m_actBaseVel, double dt) {
    if(m_qAct.data.length() == this->robot->numJoints()){
      if(rtm_data_tools::isAllFinite(m_qAct.data)){
        for(int i=0;i<m_qAct.data.length();i++){
          this->robot->joint(i)->q() = m_qAct.data[i];
        }
      }else{
        std::cerr << "m_qAct is not finite!" << std::endl;
      }
    }
    if(m_dqAct.data.length() == this->robot->numJoints()){
      if(rtm_data_tools::isAllFinite(m_dqAct.data)){
        for(int i=0;i<m_dqAct.data.length();i++){
          this->robot->joint(i)->dq() = m_dqAct.data[i];
        }
      }else{
        std::cerr << "m_dqAct is not finite!" << std::endl;
      }
    }
    if(rtm_data_tools::isAllFinite(m_actBasePose.data)){
      eigen_rtm_conversions::poseRTMToEigen(m_actBasePose.data,this->robot->rootLink()->T());
    }else{
      std::cerr << "m_actBasePose is not finite!" << std::endl;
    }
    if(rtm_data_tools::isAllFinite(m_actBaseVel.data)){
      eigen_rtm_conversions::velocityRTMToEigen(m_actBaseVel.data,this->robot->rootLink()->v(),this->robot->rootLink()->w());
    }else{
      std::cerr << "m_actBaseVel is not finite!" << std::endl;
    }

    this->robot->rootLink()->dv().setZero();
    this->robot->rootLink()->dw().setZero();
    for(int i=0;i<this->robot->numJoints();i++) {
      this->robot->joint(i)->ddq() = 0.0;
    }
    for(int i=0;i<this->robot->numLinks();i++) {
      this->robot->link(i)->F_ext().setZero();
    }
    this->robot->calcForwardKinematics(true,true);
    this->robot->calcCenterOfMass();
    cnoid::Vector6 F_o = cnoid::calcInverseDynamics(this->robot->rootLink()); // world frame origin
    this->robot->rootLink()->F_ext().head<3>() = F_o.head<3>(); // rootLink origin
    this->robot->rootLink()->F_ext().tail<3>() = F_o.tail<3>() + (-this->robot->rootLink()->p()).cross(F_o.head<3>()); // rootLink origin

    {
      Eigen::MatrixXd CMJ;
      cnoid::calcCMJacobian(this->robot,nullptr,CMJ); // [joint root]の順
      cnoid::VectorX dq(this->robot->numJoints()+6);
      for(int i=0;i<this->robot->numJoints();i++) dq[i] = this->robot->joint(i)->dq();
      dq.segment<3>(this->robot->numJoints()) = this->robot->rootLink()->v();
      dq.tail<3>() = this->robot->rootLink()->w();
      this->cogVel.passFilter(CMJ * dq, dt);
      //this->cogVel = CMJ * dq;
    }
  }

  void State::updateContactFromIdl(const contact_state_msgs::TimedContactSeq& m_actContactState){
    this->contacts.resize(m_actContactState.data.length());
    int numContact= 0;
    for(int i=0;i<m_actContactState.data.length();i++){
      if(!this->contacts[numContact]) this->contacts[numContact] = std::make_shared<Contact>();
      if(this->linkNameMap.find(std::string(m_actContactState.data[i].link1)) == this->linkNameMap.end()){
        std::cerr << __FUNCTION__ << m_actContactState.data[i].link1 << " not found" << std::endl;
        continue;
      }
      this->contacts[numContact]->link1 = this->linkNameMap[std::string(m_actContactState.data[i].link1)];
      if(!rtm_data_tools::isAllFinite(m_actContactState.data[i].local_pose)){
        std::cerr << __FUNCTION__ << "local_pose not finite" << std::endl;
        continue;
      }
      eigen_rtm_conversions::poseRTMToEigen(m_actContactState.data[i].local_pose, this->contacts[numContact]->localPose1);
      if(this->linkNameMap.find(std::string(m_actContactState.data[i].link2)) == this->linkNameMap.end()){
        std::cerr << __FUNCTION__ << m_actContactState.data[i].link2 << " not found" << std::endl;
        continue;
      }
      this->contacts[numContact]->link2 = this->linkNameMap[std::string(m_actContactState.data[i].link2)];
      this->contacts[numContact]->freeX = m_actContactState.data[i].free_x;
      this->contacts[numContact]->freeY = m_actContactState.data[i].free_y;
      for(int j=0;j<3;j++) this->contacts[numContact]->force[j] = m_actContactState.data[i].force[j];
      numContact++;
    }
    this->contacts.resize(numContact);
  }

};



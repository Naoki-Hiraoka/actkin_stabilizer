#include "ActToGenFrameConverter.h"
#include "CnoidBodyUtil.h"
#include "MathUtil.h"
#include <cnoid/EigenUtil>
#include <cnoid/RateGyroSensor>

bool ActToGenFrameConverter::convertFrame(const GaitParam& gaitParam, double dt, // input
                    std::shared_ptr<Object>& robot, std::vector<std::shared_ptr<Object> >& activeObjects) const // input & output
{
  /*
    actrobotRawのqをそのままactRobotへ. dqもfilterしてからactRobotへ

    actRobotのXYZrpy. objectsのXYZrpy, qを探索変数として、
    0 imu linkのroll pitch
    1 contactsのlocalPose1とlocalPose2のS成分 (robotがからまない)
    2 contactsのlocalPose1とlocalPose2のS成分 (robotがからむ)
    が一致するようなものを求める.

    actRobotのv, w, cogVel. objectsのdq, v, w, cogVelを、filterによって求める.
  */

  cnoid::Vector3 prevCog = robot->body->centerOfMass();
  std::vector<cnoid::Vector3> prevObjectCog(activeObjects.size());
  for(int i=0;i<activeObjects.size();i++) prevObjectCog[i] = activeObjects[i]->body->centerOfMass();

  // actrobotRawのqをそのままactRobotへ. dqもfilterしてからactRobotへ
  for(int i=0;i<gaitParam.actRobotRaw->numJoints();i++){
    robot->body->joint(i)->q() = gaitParam.actRobotRaw->joint(i)->q();
    robot->body->joint(i)->dq() = robot->dqAct[i].passFilter(gaitParam.actRobotRaw->joint(i)->dq(), dt);
    robot->body->joint(i)->ddq() = 0.0;
  }

  /*
    actRobotのXYZrpy. objectsのXYZrpy, qを探索変数として、
    0 imu linkのroll pitch
    1 contactsのlocalPose1とlocalPose2のS成分 (robotがからまない)
    2 contactsのlocalPose1とlocalPose2のS成分 (robotがからむ)
    が一致するようなものを求める.
  */

  // actRobotのXYZrpy. objectsのXYZrpy, qを探索変数とする
  std::vector<cnoid::LinkPtr> variables; variables.reserve(1+activeObjects.size());
  std::vector<double> dqWeight; dqWeight.reserve(6+6*activeObjects.size());
  variables.push_back(robot->body->rootLink());
  for(int i=0;i<6;i++) dqWeight.push_back(1.0);
  for(int j=0;j<activeObjects.size();j++){
    variables.push_back(activeObjects[j]->body->rootLink());
    for(int i=0;i<6;i++) dqWeight.push_back(1.0);
    for(size_t i=0;i<activeObjects[j]->body->numJoints();i++){
      variables.push_back(activeObjects[j]->body->joint(i));
      dqWeight.push_back(1.0);
    }
  }

  std::vector<std::shared_ptr<IK::IKConstraint> > ikConstraint0;
  std::vector<std::shared_ptr<IK::IKConstraint> > ikConstraint1;
  std::vector<std::shared_ptr<IK::IKConstraint> > ikConstraint2;

  // imu
  {
    cnoid::RateGyroSensorPtr imu = robot->body->findDevice<cnoid::RateGyroSensor>("gyrometer");
    this->imuConstraint->A_link() = imu->link();
    this->imuConstraint->A_localpos() = imu->R_local();
    cnoid::RateGyroSensorPtr imuRaw = gaitParam.actRobotRaw->findDevice<cnoid::RateGyroSensor>("gyrometer");
    this->imuConstraint->B_link() = imuRaw->link();
    this->imuConstraint->B_localpos() = imuRaw->R_local();
    this->imuConstraint->eval_link() = imuRaw->link();
    this->imuConstraint->eval_localR() = imuRaw->link()->R().transpose() * cnoid::Matrix3::Identity(); // world系
    this->imuConstraint->maxError() << 10.0*dt, 10.0*dt, 10.0*dt, 10.0*dt, 10.0*dt, 10.0*dt;
    this->imuConstraint->precision() << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0; // 強制的にIKをmax loopまで回す
    this->imuConstraint->weight() << 0.0, 0.0, 0.0, 1.0, 1.0, 0.0; // roll pitch
    ikConstraint0.push_back(this->imuConstraint);
  }

  // contact
  for(int i=0;i<gaitParam.activeContacts.size();i++){
    gaitParam.activeContacts[i]->ikPositionConstraint->A_link() = gaitParam.activeContacts[i]->link1;
    gaitParam.activeContacts[i]->ikPositionConstraint->A_localpos() = gaitParam.activeContacts[i]->localPose1;
    gaitParam.activeContacts[i]->ikPositionConstraint->B_link() = gaitParam.activeContacts[i]->link2;
    gaitParam.activeContacts[i]->ikPositionConstraint->B_localpos() = gaitParam.activeContacts[i]->localPose2.value();
    gaitParam.activeContacts[i]->ikPositionConstraint->eval_link() = gaitParam.activeContacts[i]->link1;
    gaitParam.activeContacts[i]->ikPositionConstraint->eval_localR() = gaitParam.activeContacts[i]->localPose1.linear();
    gaitParam.activeContacts[i]->ikPositionConstraint->maxError() << 10.0*dt, 10.0*dt, 10.0*dt, 10.0*dt, 10.0*dt, 10.0*dt;
    gaitParam.activeContacts[i]->ikPositionConstraint->precision() << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0; // 強制的にIKをmax loopまで回す
    gaitParam.activeContacts[i]->ikPositionConstraint->weight() << gaitParam.activeContacts[i]->axis;
    if(gaitParam.activeContacts[i]->link1->body() == robot->body || gaitParam.activeContacts[i]->link2->body() == robot->body){
      ikConstraint2.push_back(gaitParam.activeContacts[i]->ikPositionConstraint);
    }else{
      ikConstraint1.push_back(gaitParam.activeContacts[i]->ikPositionConstraint);
    }
  }

  std::vector<std::vector<std::shared_ptr<IK::IKConstraint> > > constraints{ikConstraint0,ikConstraint1,ikConstraint2};
  for(size_t i=0;i<constraints.size();i++){
    for(size_t j=0;j<constraints[i].size();j++){
      constraints[i][j]->debuglevel() = 0;//debuglevel
    }
  }
  prioritized_inverse_kinematics_solver::IKParam param;
  param.maxIteration = 1;
  param.dqWeight = dqWeight;
  param.wn = 1e-6;
  param.we = 1e0;
  param.debugLevel = 0;
  param.dt = dt;
  prioritized_inverse_kinematics_solver::solveIKLoop(variables,
                                                     constraints,
                                                     this->ikTasks,
                                                     param
                                                     );

  // actRobotのdv, dw, cogVel. objectsのdq, dv, dw, cogVelを、filterによって求める.
  robot->body->rootLink()->v() = robot->actRootv.passFilter(robot->body->rootLink()->v(), dt);
  robot->body->rootLink()->w() = robot->actRootw.passFilter(robot->body->rootLink()->w(), dt);
  robot->body->rootLink()->dv().setZero();
  robot->body->rootLink()->dw().setZero();
  robot->body->calcForwardKinematics(true, true);
  robot->body->calcCenterOfMass();
  for(int i=0;i<activeObjects.size();i++){
    activeObjects[i]->body->rootLink()->v() = activeObjects[i]->actRootv.passFilter(activeObjects[i]->body->rootLink()->v(), dt);
    activeObjects[i]->body->rootLink()->w() = activeObjects[i]->actRootw.passFilter(activeObjects[i]->body->rootLink()->w(), dt);
    activeObjects[i]->body->rootLink()->dv().setZero();
    activeObjects[i]->body->rootLink()->dw().setZero();
    for(int i=0;i<activeObjects[i]->body->numJoints();i++){
      activeObjects[i]->body->joint(i)->dq() = activeObjects[i]->dqAct[i].passFilter(activeObjects[i]->body->joint(i)->dq(), dt);
      activeObjects[i]->body->joint(i)->ddq() = 0.0;
    }
    activeObjects[i]->body->calcForwardKinematics(true, true);
    activeObjects[i]->body->calcCenterOfMass();
  }

  if(!this->isInitial){
    // Body::calcTotalMomentumを使ってもいいが、こっちのほうが計算コストが安そう
    cnoid::Vector3 cogVel = (robot->body->centerOfMass() - prevCog) / dt;
    robot->actCogVel.passFilter(cogVel, dt);
    for(int i=0;i<activeObjects.size();i++){
      cnoid::Vector3 cogVel = (activeObjects[i]->body->centerOfMass() - prevObjectCog[i]) / dt;
      activeObjects[i]->actCogVel.passFilter(cogVel, dt);
    }
  }

  this->isInitial = false;
  return true;
}

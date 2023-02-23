#ifndef WrenchDistributor_H
#define WrenchDistributor_H

#include "GaitParam.h"
#include <prioritized_qp_osqp/prioritized_qp_osqp.h>
#include <cnoid/JointPath>

#include <aik_constraint/PositionConstraint.h>
#include <aik_constraint/COMConstraint.h>
#include <aik_constraint/JointAngleConstraint.h>
#include <aik_constraint/AngularMomentumConstraint.h>
#include <aik_constraint_joint_limit_table/JointLimitMinMaxTableConstraint.h>
#include <aik_constraint/ClientCollisionConstraint.h>
#include <prioritized_acc_inverse_kinematics_solver/PrioritizedAccInverseKinematicsSolver.h>

class WrenchDistributor{
public:
  // WrenchDistributorでしか使わないパラメータ
  double joint_K = 1.0; // 0以上
  double joint_D = 1.0; // 0以上
  std::vector<cpp_filters::TwoPointInterpolator<double> > aikdqWeight; // 要素数と順序はrobot->numJoints()と同じ. 0より大きい. 各関節の速度に対するダンピング項の比. default 1. 動かしたくない関節は大きくする. 全く動かしたくないなら、controllable_jointsを使うこと. resolved acceleration control用

  void init(GaitParam& gaitParam){
    aikdqWeight.resize(gaitParam.robot->body->numJoints(), cpp_filters::TwoPointInterpolator<double>(1.0, 0.0, 0.0, cpp_filters::HOFFARBIB));

    aikEEPositionConstraint.clear();
    aikRefJointAngleConstraint.clear();
    aikJointLimitConstraint.clear();
    for(int i=0;i<gaitParam.robot->body->numJoints();i++) aikJointLimitConstraint.push_back(std::make_shared<aik_constraint_joint_limit_table::JointLimitMinMaxTableConstraint>());
    // selfCollisionConstraint.clear();
    // for(int i=0;i<gaitParam.selfCollision.size();i++) selfCollisionConstraint.push_back(std::make_shared<aik_constraint::ClientCollisionConstraint>());
  }

  // StartStabilizer時に一回呼ばれる
  void onStartStabilizer(){
  }

  // 毎周期呼ばれる
  void onExecute(double dt){
    for(int i=0;i<aikdqWeight.size();i++) aikdqWeight[i].interpolate(dt);
  }

protected:
  // 計算高速化のためのキャッシュ. クリアしなくても別に副作用はない.
  mutable std::shared_ptr<prioritized_qp_osqp::Task> constraintTask_ = std::make_shared<prioritized_qp_osqp::Task>();
  mutable std::shared_ptr<prioritized_qp_osqp::Task> tgtForceTask_ = std::make_shared<prioritized_qp_osqp::Task>();
  mutable std::shared_ptr<prioritized_qp_osqp::Task> tgtTorqueTask_ = std::make_shared<prioritized_qp_osqp::Task>();
  mutable std::shared_ptr<prioritized_qp_osqp::Task> normTask_ = std::make_shared<prioritized_qp_osqp::Task>();

  // 内部にヤコビアンの情報をキャッシュするが、クリアしなくても副作用はあまりない
  // aik
  mutable std::vector<std::shared_ptr<aik_constraint::PositionConstraint> > aikEEPositionConstraint; // 要素数と順序はendEffectorsと同じ.
  mutable std::vector<std::shared_ptr<aik_constraint::JointAngleConstraint> > aikRefJointAngleConstraint; // 要素数と順序はrobot->numJoints()と同じ
  mutable std::shared_ptr<aik_constraint::PositionConstraint> aikRootPositionConstraint = std::make_shared<aik_constraint::PositionConstraint>();
  mutable std::shared_ptr<aik_constraint::COMConstraint> aikComConstraint = std::make_shared<aik_constraint::COMConstraint>();
  mutable std::shared_ptr<aik_constraint::AngularMomentumConstraint> aikAngularMomentumConstraint = std::make_shared<aik_constraint::AngularMomentumConstraint>();
  mutable std::vector<std::shared_ptr<aik_constraint_joint_limit_table::JointLimitMinMaxTableConstraint> > aikJointLimitConstraint;
  mutable std::vector<std::shared_ptr<aik_constraint::ClientCollisionConstraint> > aikSelfCollisionConstraint;
  mutable std::vector<std::shared_ptr<prioritized_qp_base::Task> > aikTasks;

public:
  bool execWrenchDistributor(const GaitParam& gaitParam, double dt,
                             std::shared_ptr<Object>& robot) const;

protected:
  // bool calcCogAcc(const GaitParam& gaitParam, double dt, bool useActState,
  //                 GaitParam::DebugData& debugData, //for Log
  //                 cnoid::Vector3& o_tgtCogAcc/*generate座標系*/, cnoid::Vector3& o_genNextCog, cnoid::Vector3& o_genNextCogVel, cnoid::Vector3& o_genNextCogAcc, cnoid::Vector3& o_genNextForce) const;
  // bool calcZmp(const GaitParam& gaitParam, const cnoid::Vector3& cog, const cnoid::Vector3& DCM, const std::vector<cnoid::Position>& EEPose, const bool& useSoftLimit,
  //              cnoid::Vector3& o_zmp) const;
  // bool calcResolvedAccelerationControl(const GaitParam& gaitParam, double dt, const cnoid::Vector3& tgtCogAcc/*generate座標系*/, const cnoid::Vector3& genNextCog, bool useActState,
  //                                      cnoid::BodyPtr& actRobotTqc, cnoid::BodyPtr& o_genRobot) const;
  // bool calcWrench(const GaitParam& gaitParam, const cnoid::Vector3& genNextForce, bool useActState,
  //                 std::vector<cnoid::Vector6>& o_tgtEEWrench /* 要素数EndEffector数. generate座標系. EndEffector origin*/, cnoid::BodyPtr& actRobotTqc) const;
};

#endif

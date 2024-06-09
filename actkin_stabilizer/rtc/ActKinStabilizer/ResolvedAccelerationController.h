#ifndef ResolvedAccelerationController_H
#define ResolvedAccelerationController_H

#include "State.h"
#include "Goal.h"
#include <prioritized_qp_osqp/prioritized_qp_osqp.h>
#include <cnoid/JointPath>

#include <aik_constraint/PositionConstraint.h>
#include <aik_constraint/COMConstraint.h>
#include <aik_constraint/JointAngleConstraint.h>
#include <aik_constraint/AngularMomentumConstraint.h>
#include <aik_constraint_joint_limit_table/JointLimitMinMaxTableConstraint.h>
#include <aik_constraint/ClientCollisionConstraint.h>
#include <prioritized_acc_inverse_kinematics_solver/PrioritizedAccInverseKinematicsSolver.h>

namespace actkin_stabilizer {

  class ResolvedAccelerationController{
  public:
    // ResolvedAccelerationControllerでしか使わないパラメータ
    std::vector<cpp_filters::TwoPointInterpolator<double> > aikdqWeight; // 要素数と順序はrobot->numJoints()と同じ. 0より大きい. 各関節の速度に対するダンピング項の比. default 1. 動かしたくない関節は大きくする. 全く動かしたくないなら、controllable_jointsを使うこと. resolved acceleration control用
    int debugLevel = 0;

    void init(State& state);


    // StartStabilizer時に一回呼ばれる
    void onStartStabilizer();

  protected:
    // 計算高速化のためのキャッシュ. 初期化しなくてもよい
    mutable std::vector<std::shared_ptr<prioritized_qp_base::Task> > prevTasks;

  public:
    bool execResolvedAccelerationController(const State& state, const Goal& goal, const std::string& instance_name, double dt) const;

  protected:
    bool calcContactState(const State& state,
                          const Goal& goal,
                          const std::string& instance_name,
                          std::vector<std::shared_ptr<RefContact> >& allNextContacts,
                          std::vector<std::shared_ptr<Contact> >& redundantContacts) const;

    bool calcVariables(const State& state,
                       const std::vector<std::shared_ptr<RefContact> >& activeNextContacts,
                       const std::string& instance_name,
                       std::vector<cnoid::LinkPtr>& joints,
                       std::vector<std::shared_ptr<aik_constraint::Force> >& forces,
                       std::vector<std::shared_ptr<aik_constraint::IKConstraint> >& jointLimitConstraints,
                       std::vector<std::shared_ptr<aik_constraint::IKConstraint> >& forceConstraints) const;

    bool calcEOMConstraints(const State& state,
                            const std::string& instance_name,
                            std::vector<std::shared_ptr<aik_constraint::IKConstraint> >& eomConstraints) const;

    bool calcPenetrationConstraints(const State& state,
                                    const std::vector<std::shared_ptr<Contact> >& redundantContacts,
                                    const std::string& instance_name,
                                    std::vector<std::shared_ptr<aik_constraint::IKConstraint> >& penetrationConstraints) const;

    bool calcKeepContactConstraints(const State& state,
                                    const std::vector<std::shared_ptr<RefContact> >& allNextContacts,
                                    const std::string& instance_name,
                                    std::vector<std::shared_ptr<aik_constraint::IKConstraint> >& keepContactConstraints) const;

    bool calcCollisionAvoidanceConstraints(const State& state,
                                           const std::string& instance_name,
                                           std::vector<std::shared_ptr<aik_constraint::IKConstraint> >& collisionAvoidanceConstraints) const;

    bool calcCOMConstraints(const State& state,
                            const Goal& goals,
                            const std::vector<std::shared_ptr<aik_constraint::Force> >& forces,
                            const std::vector<std::shared_ptr<aik_constraint::IKConstraint> >& forceConstraints,
                            const std::vector<std::shared_ptr<aik_constraint::IKConstraint> >& eomConstraints,
                            const std::string& instance_name,
                            std::vector<std::shared_ptr<aik_constraint::IKConstraint> >& comConstraints) const;

    bool calcEEFConstraints(const State& state,
                            const Goal& goals,
                            const std::string& instance_name,
                            std::vector<std::shared_ptr<aik_constraint::IKConstraint> >& eefHighConstraints,
                            std::vector<std::shared_ptr<aik_constraint::IKConstraint> >& eefLowConstraints) const;

    bool calcJointConstraints(const State& state,
                              const Goal& goals,
                              const std::string& instance_name,
                              std::vector<std::shared_ptr<aik_constraint::IKConstraint> >& jointAngleConstraints,
                              std::vector<std::shared_ptr<aik_constraint::IKConstraint> >& angularMomentumConstraints) const;

    bool calcRAC(const std::vector<std::shared_ptr<aik_constraint::IKConstraint> >& jointAngleLimitConstraints,
                 const std::vector<std::shared_ptr<aik_constraint::IKConstraint> >& forceConstraints,
                 const std::vector<std::shared_ptr<aik_constraint::IKConstraint> >& eomConstraints,
                 const std::vector<std::shared_ptr<aik_constraint::IKConstraint> >& penetrationConstraints,
                 const std::vector<std::shared_ptr<aik_constraint::IKConstraint> >& keepContactConstraints,
                 const std::vector<std::shared_ptr<aik_constraint::IKConstraint> >& collisionAvoidanceConstraints,
                 const std::vector<std::shared_ptr<aik_constraint::IKConstraint> >& comConstraints,
                 const std::vector<std::shared_ptr<aik_constraint::IKConstraint> >& eefHighConstraints,
                 const std::vector<std::shared_ptr<aik_constraint::IKConstraint> >& eefLowConstraints,
                 const std::vector<std::shared_ptr<aik_constraint::IKConstraint> >& jointAngleConstraints,
                 const std::vector<std::shared_ptr<aik_constraint::IKConstraint> >& angularMomentumConstraints,
                 const std::string& instance_name,
                 const std::vector<cnoid::LinkPtr>& joints,
                 const std::vector<std::shared_ptr<aik_constraint::Force> >& forces) const;

    bool calcTorque(const State& state,
                    const std::vector<std::shared_ptr<RefContact> >& activeNextContacts,
                    const std::string& instance_name) const;

    // bool calcCogAcc(const State& gaitParam, double dt, bool useActState,
    //                 State::DebugData& debugData, //for Log
    //                 cnoid::Vector3& o_tgtCogAcc/*generate座標系*/, cnoid::Vector3& o_genNextCog, cnoid::Vector3& o_genNextCogVel, cnoid::Vector3& o_genNextCogAcc, cnoid::Vector3& o_genNextForce) const;
    // bool calcZmp(const State& gaitParam, const cnoid::Vector3& cog, const cnoid::Vector3& DCM, const std::vector<cnoid::Position>& EEPose, const bool& useSoftLimit,
    //              cnoid::Vector3& o_zmp) const;
    // bool calcResolvedAccelerationControl(const State& gaitParam, double dt, const cnoid::Vector3& tgtCogAcc/*generate座標系*/, const cnoid::Vector3& genNextCog, bool useActState,
    //                                      cnoid::BodyPtr& actRobotTqc, cnoid::BodyPtr& o_genRobot) const;
    // bool calcWrench(const State& gaitParam, const cnoid::Vector3& genNextForce, bool useActState,
    //                 std::vector<cnoid::Vector6>& o_tgtEEWrench /* 要素数EndEffector数. generate座標系. EndEffector origin*/, cnoid::BodyPtr& actRobotTqc) const;
  };
};

#endif

#ifndef GOAL_H
#define GOAL_H

#include <unordered_map>
#include <Eigen/SparseCore>
#include <cpp_filters/cpp_filters.h>
#include <actkin_stabilizer_msgs/idl/ActKinStabilizer.hh>
#include <aik_constraint/aik_constraint.h>
#include "State.h"

namespace actkin_stabilizer {
  class Region3D {
  public:
    Eigen::MatrixXd C = Eigen::MatrixXd::Identity(3,3);
    cnoid::VectorX ld = cnoid::VectorX::Zero(3);
    cnoid::VectorX ud = cnoid::VectorX::Zero(3);
  };

  class RefEE {
  public:
    std::string name;
    cnoid::LinkPtr link;
    cnoid::Isometry3 localPose = cnoid::Isometry3::Identity();

    cnoid::LinkPtr frameLink;
    cnoid::Isometry3 framePose = cnoid::Isometry3::Identity();
    //Region3D posRegion;
    std::vector<bool> freeAxis = std::vector<bool>(6,true);
    int priority = 1; // 0 or 1. 0ならふつう. 1は重心と同じ
    std::vector<cpp_filters::TwoPointInterpolatorSE3> pose; // 必ずサイズは1以上
    std::vector<cpp_filters::TwoPointInterpolator<cnoid::Vector6> > wrench; // 必ずサイズは1以上

    std::shared_ptr<aik_constraint::PositionConstraint> positionConstraint = nullptr;
  };

  class RefVRP {
  public:
    double omega = 1.0;
    std::vector<cpp_filters::TwoPointInterpolator<cnoid::Vector3> > vrp; // 必ずサイズは1以上

    std::shared_ptr<aik_constraint::COMConstraint> comConstraint = nullptr;
    std::shared_ptr<aik_constraint::Force> force = nullptr;
    std::shared_ptr<aik_constraint::ForceConstraint> forceConstraint = nullptr;
    std::shared_ptr<aik_constraint::Force> force2 = nullptr;
    std::shared_ptr<aik_constraint::ForceConstraint> force2Constraint1 = nullptr;
    std::shared_ptr<aik_constraint::ForceConstraint> force2Constraint2 = nullptr;
    std::shared_ptr<aik_constraint::AngularMomentumConstraint> angularMomentumConstraint = nullptr;
  };

  class Refq {
  public:
    std::vector<cpp_filters::TwoPointInterpolator<cnoid::VectorX> > q; // 必ずサイズは1以上

    std::vector<std::shared_ptr<aik_constraint::JointAngleConstraint> > jointAngleConstraints;
  };

  class RefContact {
  public:
    std::string name;
    cnoid::LinkPtr link1;
    cnoid::Isometry3 localPose1 = cnoid::Isometry3::Identity();
    cnoid::LinkPtr link2;
    std::vector<bool> freeAxis = std::vector<bool>(6,false); // localPose1 local
    Region3D region; // localPose1 local

    double muTrans = 0.5; // 0以上
    double muRot = 0.05; // 0以上
    double maxFz = 2000.0; // 0以上
    double minFz = 50.0; // 0以上
    std::vector<Eigen::Vector2d> surface = std::vector<Eigen::Vector2d>{Eigen::Vector2d(0.05,0.05),Eigen::Vector2d(-0.05,0.05),Eigen::Vector2d(-0.05,-0.05),Eigen::Vector2d(0.05,-0.05)}; // 半時計回り. 面積が0でない

    std::shared_ptr<aik_constraint::Force> force = nullptr;
    std::shared_ptr<aik_constraint::ForceConstraint> forceConstraint = nullptr;
    std::shared_ptr<aik_constraint::ForceConstraint> forceConstraint2 = nullptr;
    std::shared_ptr<aik_constraint::ForceConstraint> forceReductionConstraint = nullptr;
    std::shared_ptr<aik_constraint::PositionConstraint> positionConstraint = nullptr;

  };

  class Goal {
  public:
    // from port
    std::unordered_map<std::string, std::shared_ptr<RefEE> > eeGoals;
    std::vector<std::shared_ptr<RefVRP> > vrpGoals;
    std::vector<std::shared_ptr<Refq> > qGoals;
    std::unordered_map<std::string, std::shared_ptr<RefContact> > contactGoals;

  public:
    // parameter
    double minHorizonTime = 0.3;
    double Kp = 300.0;
    double Dp = 35.0;
    double Kr = 200.0;
    double Dr = 30.0;
    double Kq = 10.0;
    double Dq = 10.0;

    double contactDp = 1.0; // rootのvelフィルタのため支持脚は速度を持つので、15だと悪さをする
    double contactDr = 5.0; // 30.0だと斜面で縁が接触したときに面接触に移行しない. 15だとバタつく. 5?

    double contactMargin = 0.02; // 接触点から離れすぎた位置にCOPを出力すると、接触点を中心に旋回するのではなく空中で旋回してしまい意図せぬ挙動となる. 近すぎる位置にしか出力しないと、edge接触から面接触への移行に時間がかかる. 0.05だと大きすぎる. 0.01だと小さすぎる

    double forceRatio = 1e-2; // 100N = 100kg*1ms/s^2と1m/s^2を同じ最適化で扱うためにスケーリング. これがないと加速度の誤差が大きくなり、特に動歩行時の重心の加速が問題になる.

    cnoid::VectorX dqWeight; // サイズはstate.robot->numJointsと同じ
  public:
    // RTC起動時に一回呼ばれる.
    void init(const State& state);

    // startStabilizer時に呼ばれる
    void onStartStabilizer();

    // MODE_ST中のみ呼ばれる
    void updateFromIdl(const State& state, const actkin_stabilizer_msgs::RefStateIdl& m_refState);
    void updateFromIdl(const State& state, const actkin_stabilizer_msgs::RefEESequence& m_refEEPose);
    void updateFromIdl(const State& state, const actkin_stabilizer_msgs::RefVRPIdl& m_refVRP);
    void updateFromIdl(const State& state, const actkin_stabilizer_msgs::RefqIdl& m_refq);
    void updateFromIdl(const State& state, const actkin_stabilizer_msgs::RefContactSequence& m_refContact);

    // MODE_ST中のみ呼ばれる. 各goalをdtだけ補間する.
    void interpolate(double dt);
  };
};

#endif

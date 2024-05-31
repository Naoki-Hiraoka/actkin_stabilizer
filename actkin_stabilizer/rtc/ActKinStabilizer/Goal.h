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

  class Task {
  public:
    // 干渉回避など
    virtual const std::vector<std::shared_ptr<aik_constraint::Constraint> >& firstPriorityTasks() {
      return std::vector<std::shared_ptr<aik_constraint::Constraint> >();
    }
    // エンドエフェクタなど
    virtual const std::vector<std::shared_ptr<aik_constraint::Constraint> >& secondPriorityTasks() {
      return std::vector<std::shared_ptr<aik_constraint::Constraint> >();
    }
    // 関節角度など
    virtual const std::vector<std::shared_ptr<aik_constraint::Constraint> >& thildPriorityTasks() {
      return std::vector<std::shared_ptr<aik_constraint::Constraint> >();
    }
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
  public:
    bool interpolate(double dt);
  };

  class RefVRP {
  public:
    double omega = 1.0;
    std::vector<cpp_filters::TwoPointInterpolator<cnoid::Vector3> > vrp; // 必ずサイズは1以上
  public:
    bool interpolate(double dt);
  };

  class Refq {
  public:
    std::vector<cpp_filters::TwoPointInterpolator<cnoid::VectorX> > q; // 必ずサイズは1以上
  public:
    Refq();
    bool interpolate(double dt);
  };

  class RefContact {
  public:
    std::string name;
    cnoid::LinkPtr link1;
    cnoid::Isometry3 localPose1 = cnoid::Isometry3::Identity();
    cnoid::LinkPtr link2;
    std::vector<bool> freeAxis = std::vector<bool>(6,true); // localPose1 local
    Region3D region; // localPose1 local

    Eigen::SparseMatrix<double,Eigen::RowMajor> wrenchC; // localPose1 frame/origin. link1がlink2から受ける力に関する接触力制約.
    cnoid::VectorX wrenchld;
    cnoid::VectorX wrenchud;
  };

  class Goal {
  public:
    std::unordered_map<std::string, std::shared_ptr<RefEE> > eeGoals;
    std::vector<std::shared_ptr<RefVRP> > vrpGoals;
    std::vector<std::shared_ptr<Refq> > qGoals;
    std::unordered_map<std::string, std::shared_ptr<RefContact> > contactGoals;

  public:
    // RTC起動時に一回呼ばれる.
    void init(const State& state);

    // MODE_ST中のみ呼ばれる
    void updateFromIdl(const State& state, const actkin_stabilizer_msgs::RefStateIdl& m_refState);
    void updateFromIdl(const State& state, const actkin_stabilizer_msgs::RefEESequence& m_refEEPose);
    void updateFromIdl(const State& state, const actkin_stabilizer_msgs::RefVRPIdl& m_refVRP);
    void updateFromIdl(const State& state, const actkin_stabilizer_msgs::RefqIdl& m_refq);
    void updateFromIdl(const State& state, const actkin_stabilizer_msgs::RefContactSequenceIdl& m_refContact);

    // startStabilizer時に呼ばれる
    void onStartStabilizer();

    // MODE_ST中のみ呼ばれる. 各goalをdtだけ補間する.
    void interpolate(double dt);
  };
};

#endif

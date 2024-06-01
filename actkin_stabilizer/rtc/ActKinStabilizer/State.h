#ifndef STATE_H
#define STATE_H

#include <sys/time.h>
#include <cnoid/EigenTypes>
#include <vector>
#include <memory>
#include <unordered_map>
#include <limits>
#include <cpp_filters/TwoPointInterpolator.h>
#include <cpp_filters/FirstOrderLowPassFilter.h>
#include <joint_limit_table/JointLimitTable.h>
#include <ik_constraint2/PositionConstraint.h>
#include "FootGuidedController.h"
#include <actkin_stabilizer/idl/ActKinStabilizerService.hh>
#include <contact_state_msgs/idl/ContactState.hh>


namespace actkin_stabilizer {

  class Contact {
  public:
    cnoid::LinkPtr link1;
    cnoid::Isometry3 localPose1 = cnoid::Isometry3::Identity();
    cnoid::LinkPtr link2;
    bool freeX = false;
    bool freeY = false;
    //std::shared_ptr<ik_constraint2::PositionConstraint> ikc;
  };

  class Collision {
  public:
    cnoid::LinkPtr link1; // world model
    cnoid::Vector3 point1 = cnoid::Vector3::Zero(); // link1 frame
    cnoid::LinkPtr link2; // world model
    cnoid::Vector3 point2 = cnoid::Vector3::Zero(); // link2 frame
    cnoid::Vector3 direction21 = cnoid::Vector3::UnitX(); // generate frame
    double distance = 0.0;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  class State {
    // このクラスのメンバ変数は、全てfiniteである(nanやinfが無い)ことが仮定されている. 外部から値をセットするときには、finiteでない値を入れないようにすること

  public:
    // from data port. 狭義のstate
    cnoid::BodyPtr robot; // actual.
    cpp_filters::FirstOrderLowPassFilter<cnoid::Vector3> cogVel{3.5, cnoid::Vector3::Zero()}; // generate frame.  現在のCOM速度. cutoff=4.0Hzは今の歩行時間と比べて遅すぎる気もするが、実際のところ問題なさそう? もとは4Hzだったが、 静止時に衝撃が加わると上下方向に左右交互に振動することがあるので少し小さくする必要がある. 3Hzにすると、追従性が悪くなってギアが飛んだ

    std::vector<std::shared_ptr<Contact> > contacts; // actual
    // objects

  public:
    // from data port. derived variable.
    std::vector<Collision> selfCollision;
    std::vector<Collision> envCollision;
    std::vector<Collision> objselfCollision;
    std::vector<Collision> objenvCollision;

  public:
    // parameters
    std::vector<double> softMaxTorque; // MODE_ST中はconstant. 要素数と順序はnumJoints()と同じ. 単位は[Nm]. 0以上. softMaxTorqueとモデルファイルの値の小さい方の値が使われる.
    std::vector<bool> jointControllable; // MODE_ST中はconstant. 要素数と順序はnumJoints()と同じ. falseの場合、RACでは動かさない(act値をそのまま). WDでは無視. トルク計算では目標トルクを通常通り計算した後、refTauの値で上書きされる.
    std::vector<std::vector<std::shared_ptr<joint_limit_table::JointLimitTable> > > jointLimitTables; // constant. 要素数と順序はnumJoints()と同じ. for robot.

    const double g = 9.80665; // constant. 重力加速度
    std::unordered_map<std::string, cnoid::LinkPtr> linkNameMap; // MODE_ST中はconstant. URDFのLink名 -> linkPtr

  public:
    // cache
    std::vector<std::shared_ptr<Contact> > activeContacts; // contactによってworldを介さずにrobotとつながっているcontacts. RACで考慮する.
    //std::vector<std::shared_ptr<Object> > activeObjects; // contactによってworldを介さずにrobotとつながっているobjects. RACで考慮する.

  public:
    // RTC起動時に一回呼ばれる.
    void init(const cnoid::BodyPtr& robot_);

    // startStabilizer時に呼ばれる
    void onStartStabilizer();

    // MODE_ST中のみ呼ばれる
    void updateRobotFromIdl(const RTC::TimedDoubleSeq& m_qAct, const RTC::TimedDoubleSeq& m_dqAct, const RTC::TimedPose3D& m_actBasePose, const RTC::TimedVelocity3D& m_actBaseVel, double dt);
    void updateContactFromIdl(const contact_state_msgs::TimedContactSeq& m_actContactState);

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // for debug
    mutable struct timeval prevTime;
    void resetTime() const { gettimeofday(&prevTime, NULL);}
    void printTime(const std::string& message="") const {
      struct timeval currentTime;
      gettimeofday(&currentTime, NULL);
      std::cerr << message << (currentTime.tv_sec - prevTime.tv_sec) + (currentTime.tv_usec - prevTime.tv_usec) * 1e-6 << std::endl;
    }
  };


    // class Object {
  // public:
  //   std::string name;
  //   cnoid::BodyPtr body; // nullptrでは無いことが保証されている. rootLinkがFreeJointでなければならない. FixedJointにしたければ、接触力無限のContactを利用せよ

  //   // ActToGenFrameConverter
  //   std::vector<cpp_filters::FirstOrderLowPassFilter<double> > dqAct; // これを使ってfilterした後の値がbody->link->dq()に入る. cutoffを2loopぶんにするために、passFilterのdtは常に1/2[s], cutOffは1[Hz]とする.
  //   cpp_filters::FirstOrderLowPassFilter<cnoid::Vector3> actRootv{3.5, cnoid::Vector3::Zero()}; // generate frame. 現在のroot速度. rootLink origin. なんとなくactCogVelと同程度のhzにしておく. これを使ってfilterした後の値がactRobot->rootLink()->v()/w()に入る
  //   cpp_filters::FirstOrderLowPassFilter<cnoid::Vector3> actRootw{3.5, cnoid::Vector3::Zero()}; // generate frame. 現在のroot速度. rootLink origin. なんとなくactCogVelと同程度のhzにしておく. これを使ってfilterした後の値がactRobot->rootLink()->v()/w()に入る
  //   cpp_filters::FirstOrderLowPassFilter<cnoid::Vector3> actCogVel{3.5, cnoid::Vector3::Zero()}; // generate frame.  現在のCOM速度. cutoff=4.0Hzは今の歩行時間と比べて遅すぎる気もするが、実際のところ問題なさそう? もとは4Hzだったが、 静止時に衝撃が加わると上下方向に左右交互に振動することがあるので少し小さくする必要がある. 3Hzにすると、追従性が悪くなってギアが飛んだ

  // public:
  //   Object(cnoid::BodyPtr body_) : // bodyはnullptrであってはならない
  //     body(body_),
  //     dqAct(body_->numJoints(), cpp_filters::FirstOrderLowPassFilter<double>(1.0, 0.0))
  //   {
  //     body->rootLink()->v().setZero(); body->rootLink()->w().setZero(); body->rootLink()->dv().setZero(); body->rootLink()->dw().setZero();
  //     for(int i=0;i<body->numJoints();i++){
  //       body->joint(i)->dq() = 0.0; body->joint(i)->ddq() = 0.0; body->joint(i)->u() = 0.0;
  //     }
  //     body->calcForwardKinematics(true, true); body->calcCenterOfMass();
  //   }

  //   void onExecute(double dt){
  //   }
  //   void onStartAutoBalancer(){
  //     for(int i=0;i<dqAct.size();i++) dqAct[i].reset(0.0);
  //     actRootv.reset(cnoid::Vector3::Zero());
  //     actRootw.reset(cnoid::Vector3::Zero());
  //     actCogVel.reset(cnoid::Vector3::Zero());
  //   }
  //   void onStartStabilizer(){
  //   }
  // };



  // enum leg_enum{RLEG=0, LLEG=1, NUM_LEGS=2};

  // class PositionEx {
  //   cnoid::Position pose = cnoid::Position::Identity();
  //   cnoid::Vector6 vel = cnoid::Vector6::Zero();
  //   cnoid::Vector6 acc = cnoid::Vector6::Zero();
  //   cnoid::Vector6 wrench = cnoid::Vector6::Zero();
  // };

  // class EndEffector {
  //   // このクラスのメンバ変数は、全てfiniteである(nanやinfが無い)ことが仮定されている. 外部から値をセットするときには、finiteでない値を入れないようにすること
  // public:
  //   // constaint parameter (NOT_CARED時にしか変更不可)

  //   std::string name = "";
  //   std::string parentLink = ""; // 親リンク. 必ずrobot->link(parentLink)がnullptrではないことを約束する. そのため、毎回robot->link(parentLink)がnullptrかをチェックしなくても良い
  //   cnoid::Position localT = cnoid::Position::Identity(); // Parent Link Frame

  //   std::string forceSensor = ""; // センサ名. actualのForceSensorの値を座標変換したものがEndEffectorが受けている力とみなされる. forceSensorが""ならば受けている力は常に0とみなされる. forceSensorが""で無いならばrobot->findDevice<cnoid::ForceSensor>(forceSensor)がnullptrでは無いことを約束するので、毎回nullptrかをチェックしなくても良い

  //   cnoid::Vector6 Kp = (cnoid::Vector6() << 50, 50, 50, 20, 20, 20).finished(); // endeffector frame. 分解加速度制御用. 0以上
  //   cnoid::Vector6 Dp = (cnoid::Vector6() << 10, 10, 10, 10, 10, 10).finished(); // endeffector frame. 分解加速度制御用. 0以上
  //   cnoid::Vector6 Kw = (cnoid::Vector6() <<  0,  0,  0,  0,  0,  0).finished(); // endeffector frame. passivity用. 0以上
  //   cnoid::Vector6 Dw = (cnoid::Vector6() <<  0,  0,  0,  0,  0,  0).finished(); // endeffector frame. passivity用. 0以上
  //   double odomWeight = 1.0; // 0より大きい.1以下
  //   Eigen::SparseMatrix<double,Eigen::RowMajor> wrenchC = Eigen::SparseMatrix<double,Eigen::RowMajor>(0,6);  //wrench の制約. ee frame. ee origin. rleg,llegは、legHullから自動計算されるので、変更不可.
  //   cnoid::VectorX wrenchld = cnoid::VectorX(0);
  //   cnoid::VectorX wrenchud = cnoid::VectorX(0);

  // public:
  //   // 変数. 制御処理中で変更される.

  //   enum class mode_enum{KEEP_PREV, // to controll only
  //                        NOT_CARED,
  //                        REL_HRPSYS_ODOM,
  //                        REL_COG,
  //                        // REL_LINK,
  //                        TO_CONTACT,
  //                        // ここより下、接触状態
  //                        TO_AIR,
  //                        CONTACT};
  //   mode_enum mode = mode_enum::NOT_CARED;
  //   unsigned long modeId = 0; // 今のmodeId未満のmodeIdが書かれたtopicが来ても無視する.
  //   /*
  //     非接触時、
  //      - ikGainのどれかが0->1になる
  //      - passivityGainのどれかが0->1になる
  //      - priorityが上がる
  //     のいずれかが発生する時、refPoseが強制的にgoActualする.
  //   */
  //   cnoid::Vector6 ikGain = cnoid::Vector6::Ones(); // endeffector frame. 0 or 1. 分解加速度制御のIKで考慮するか (非接触時のみ)
  //   enum class priority_enum{EE_MIDIUM, // 通常
  //                            EE_LOW, // 低優先度タスク
  //                            EE_HIGH}; // 歩行時の遊脚
  //   priority_enum priority = priority_enum::EE_MIDIUM; // 分解加速度制御のIKの優先度 (非接触時のみ)
  //   cnoid::Vector6 passivityGain = cnoid::Vector6::Zero(); // endeffector frame. 0 or 1. passivityに基づき力を出力するか (非接触時のみ)

  //   cpp_filters::TwoPointInterpolatorSE3 refPoseLocal = cpp_filters::TwoPointInterpolatorSE3(cnoid::Position::Identity(),cnoid::Vector6::Zero(),cnoid::Vector6::Zero(),cpp_filters::HOFFARBIB); // 座標系はmodeによって決まる
  //   cpp_filters::TwoPointInterpolator<cnoid::Vector6> refWrenchLocal = cpp_filters::TwoPointInterpolator<cnoid::Vector6>(cnoid::Vector6::Zero(),cnoid::Vector6::Zero(),cnoid::Vector6::Zero(), cpp_filters::HOFFARBIB); // 座標系はmodeによって決まる
  //   std::vector<std::pair<PositionEx, double> > refTrajectoryLocal; // first: 座標系はmodeによって決まる. second: time_from_prev. 0,1,2,3,4...と目標軌道があるときに、0がrefPoseLocalとrefWrenchLocalになり、1以降がここに入る
  //   PositionEx refPose; // generate frame. endeffector origin. refPoseLocalとrefWrenchLocalを座標変換したもの

  //   cnoid::Position actPose = cnoid::Position::Identity(); // generate frame.
  // };


  // enum class wholeBodyState_enum{BIPED, // rleg,lleg以外、接触状態のエンドエフェクタが無い
  //                                MULTI_CONTACT, // rleg,lleg以外に接触状態のエンドエフェクタがある
  //                                DYNAMIC_STEP, // 動歩行中. 移動目的でない
  //                                DYNAMIC_MOVE}; // 動歩行中. 移動目的
  // wholeBodyState_enum wholeBodyState = BIPED;

  // std::vector<EndEffector> endEffectors; // 要素数2以上. 0番目がrleg, 1番目がllegという名前である必要がある.

};

#endif

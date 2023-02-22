#ifndef GAITPARAM_H
#define GAITPARAM_H

#include <sys/time.h>
#include <cnoid/EigenTypes>
#include <vector>
#include <limits>
#include <cpp_filters/TwoPointInterpolator.h>
#include <cpp_filters/FirstOrderLowPassFilter.h>
#include <joint_limit_table/JointLimitTable.h>
#include "FootGuidedController.h"

class Object {
public:
  cnoid::BodyPtr body; // nullptrでは無いことが保証されている. rootLinkがFreeJointでなければならない. FixedJointにしたければ、接触力無限のContactを利用せよ

  // ActToGenFrameConverter
  std::vector<cpp_filters::FirstOrderLowPassFilter<double> > dqAct; // これを使ってfilterした後の値がbody->link->dq()に入る. cutoffを2loopぶんにするために、passFilterのdtは常に1/2[s], cutOffは1[Hz]とする.
  cpp_filters::FirstOrderLowPassFilter<cnoid::Vector6> actRootVel(3.5, cnoid::Vector6::Zero()); // generate frame. 現在のroot速度. rootLink origin. なんとなくactCogVelと同程度のhzにしておく. これを使ってfilterした後の値がactRobot->rootLink()->v()/w()に入る
  cpp_filters::FirstOrderLowPassFilter<cnoid::Vector3> actCogVel(3.5, cnoid::Vector3::Zero()); // generate frame.  現在のCOM速度. cutoff=4.0Hzは今の歩行時間と比べて遅すぎる気もするが、実際のところ問題なさそう? もとは4Hzだったが、 静止時に衝撃が加わると上下方向に左右交互に振動することがあるので少し小さくする必要がある. 3Hzにすると、追従性が悪くなってギアが飛んだ

public:
  Object(cnoid::BodyPtr body_) : // bodyはnullptrであってはならない
    body(body_)
    dqAct(body_->numJoints(), cpp_filters::FirstOrderLowPassFilter(1.0, 0.0))
  {
    body->rootLink()->v().setZero(); body->rootLink()->w().setZero(); body->rootLink()->dv().setZero(); body->rootLink()->dw().setZero();
    for(int i=0;i<body->numJoints();i++){
      body->joint(i)->dq() = 0.0; body->joint(i)->ddq() = 0.0; body->joint(i)->u() = 0.0;
    }
    body->calcForwardKinematics(true, true); body->calcCenterOfMass();
  }

  void onExecute(double dt){
  }
  void onStartAutoBalancer(){
    for(int i=0;i<dqAct.size();i++) dqAct[i].reset(0.0);
    actRootVel.reset(cnoid::Vector6::Zero());
    actCogVel.reset(cnoid::Vector6::Zero());
  }
  void onStartStabilizer(){
  }
};

class Contact {
public:
  // from InPort
  std::string name;
  unsigned long stateId; // 現在のstateId以外のstateIdを伴う指令は無視する
  cnoid::LinkPtr link1; // nullptrならworld
  cnoid::Position localPose1; // link1 frame.
  cnoid::LinkPtr link2; // nullptrならworld
  cnoid::Vector6 axis; // localPose1 frame/origin. 動かせない&接触力が発生する軸のみtrue
  Eigen::SparseMatrix<double,Eigen::RowMajor> C(0,0); // localPose1 frame/origin. link1がlink2から受ける力に関する接触力制約. 列はaxisがtrueの軸数に対応
  cnoid::VectorX ld;
  cnoid::VectorX ud;
  cnoid::VectorX w; // localPose1 frame/origin. link1がlink2から受ける力に関する重み. axisがtrueの軸数に対応

public:
  void onExecute(double dt){
  }
  void onStartAutoBalancer(){
    cnoid::Position pose1 = link1 ? link1->T() * localPose1 : localPose1;
    this->prevLocalPose2 = link2 ? link2->T().inverse() * pose1 : pose1;
  }
  void onStartStabilizer(){
  }

  // ActToGenFrameConverterで変更される
  cnoid::Position prevLocalPose2; // link2 frame.
};

class Attention {
public:
  // from Inport
  std::string name;
  unsigned long stateId; // 現在のstateId以外のstateIdを伴う指令は無視する
  cnoid::BodyPtr cog1; // 非nullptrならcog
  cnoid::LinkPtr link1; // nullptrならworld
  cpp_filters::TwoPointInterpolatorSE3 localPose1(cnoid::Position::Identity(),cnoid::Vector6::Zero(),cnoid::Vector6::Zero(),cpp_filters::HOFFARBIB); // link1 frame
  cpp_filters::TwoPointInterpolator<cnoid::Vector6> refWrench(cnoid::Vector6::Zero(),cnoid::Vector6::Zero(),cnoid::Vector6::Zero(),cpp_filters::HOFFARBIB); // world frame. localPose origin. link1がlocalPose1の位置で受ける力. (link2はlocalPose2の位置で反対の力を受ける)
  cnoid::BodyPtr cog2; // 非nullptrならcog
  cnoid::LinkPtr link2; // nullptrならworld
  cpp_filters::TwoPointInterpolatorSE3 localPose2(cnoid::Position::Identity(),cnoid::Vector6::Zero(),cnoid::Vector6::Zero(),cpp_filters::HOFFARBIB); // link2 frame
  Eigen::SparseMatrix<double,Eigen::RowMajor> Cp(0,6); // localPose1 frame/origin. localpose1から見たlocalpose2の位置姿勢に関する制約
  cnoid::VectorX ldp;
  cnoid::VectorX udp;
  cnoid::VectorX Kp; // - Kp * (Cp * 位置姿勢 - ldp|udp) を、分解加速度制御の目標加速度として使う
  cnoid::VectorX Dp; // - Dp * Cp * 速度 を、分解加速度制御の目標加速度として使う
  cnoid::VectorX limitp; // (Cp * 位置姿勢 - ldp|udp)に対するlimit
  cnoid::VectorX weightp; // 分解加速度制御の重み. default1.0
  long priority; // 分解加速度制御の優先度. 大きいほど高優先度
  double horizon; // [s]. 0より大きいなら、MPCになる.(world-重心のみ可)
  Eigen::SparseMatrix<double,Eigen::RowMajor> Cw(0,6); // localPose1 frame/origin. localpose1から見たlocalpose2の位置姿勢に関する制約
  cnoid::VectorX ldw;
  cnoid::VectorX udw;
  cnoid::VectorX Kw; // - Kw * (Cw * 位置姿勢 - ldw|udw) を、目標反力として使う
  cnoid::VectorX Dw; // - Dw * Cw * 速度 を、目標反力として使う
  cnoid::VectorX limitw; // (Cw * 位置姿勢 - ldw|udw)に対するlimit
  // 重心の場合、回転成分は無視する. MPCのとき、Kp[0] / Dp[0] をomegaとして使う.

public:
  void onExecute(double dt){
    this->localPose1.interpolate(dt);
    this->localPose2.interpolate(dt);
    this->refWrench.interpolate(dt);
  }
  void goActual(bool clearWrench=false){
    cnoid::Position pose1 = link1 ? link1->T() * localPose1.value() : localPose1.value();
    this->localPose2.reset(link2 ? link2->T().inverse() * pose1 : pose1);
    if(clearWrench) this->refWrench.reset(cnoid::Vector6::Zero());
  }
  void onStartAutoBalancer(bool clearWrench=false){
  }
  void onStartStabilizer(){
    this->goActual();
    this->stateId++;
  }
};

class GaitParam {
  // このクラスのメンバ変数は、全てfiniteである(nanやinfが無い)ことが仮定されている. 外部から値をセットするときには、finiteでない値を入れないようにすること

public:
  // constant parameters
  std::vector<double> maxTorque; // constant. 要素数と順序はnumJoints()と同じ. 単位は[Nm]. 0以上
  std::vector<std::vector<std::shared_ptr<joint_limit_table::JointLimitTable> > > jointLimitTables; // constant. 要素数と順序はnumJoints()と同じ. for robot.

  const double g = 9.80665; // 重力加速度

public:
  // dynamic parameters
  std::vector<cpp_filters::TwoPointInterpolator<double> > softMaxTorque; // constant. 要素数と順序はnumJoints()と同じ. 単位は[Nm]. 0以上
  std::vector<bool> jointControllable; // 要素数と順序はnumJoints()と同じ. falseの場合、RACでは動かさない(act値をそのまま). WDでは無視. トルク計算では目標トルクを通常通り計算した後、refTauの値で上書きされる.

public:
  // from data port
  cnoid::BodyPtr refRobotRaw; // reference. reference world frame
  cnoid::BodyPtr actRobotRaw; // actual. actual imu world frame
  class Collision {
  public:
    std::LinkPtr link1; // world model
    cnoid::Vector3 point1 = cnoid::Vector3::Zero(); // link1 frame
    std::LinkPtr link2; // world model
    cnoid::Vector3 point2 = cnoid::Vector3::Zero(); // link2 frame
    cnoid::Vector3 direction21 = cnoid::Vector3::UnitX(); // generate frame
    double distance = 0.0;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
  std::vector<Collision> selfCollision;
  std::vector<Collision> envCollision;
  std::vector<Collision> objselfCollision;
  std::vector<Collision> objenvCollision;

  // world model (generate frame)
  std::shared_ptr<Object> robot;
  std::unordered_map<std::string, std::shared_ptr<Object> > objects; // [name, ptr]. 配列中に同じ実体を指すptrが複数あることは無い
  std::unordered_map<std::string, std::shared_ptr<Contact> > contacts; // [name, ptr]. 配列中に同じ実体を指すptrが複数あることは無い
  std::unordered_map<std::string, std::shared_ptr<Attention> > attentions; // [name, ptr]. 配列中に同じ実体を指すptrが複数あることは無い

  // cache
  std::vector<std::shared_ptr<Object> > activeObjects; // contactによってworldを介さずにrobotとつながっているobjects. 状態推定・RAC・WDで考慮する.
  std::vector<std::vector<std::shared_ptr<Attention> > > prioritizedAttentions; // attentionsを優先度順に並び替えたもの


  // ActToGenFrameConverter
  // ResolvedAccelerationController. robotのddqの値として指令関節加速度が入る
  // WrenchDistributor. robotのuの値として指令関節トルクが入る

public:
  // for debug data
  class DebugData {
  public:
    // Stabilizer
    cnoid::Vector3 stTargetZmp; // generate frame. stで計算された目標ZMP
    std::vector<cnoid::Vector6> stEETargetWrench; // 要素数と順序はendEffectorsと同じ.generate frame. EndEffector origin. ロボットが受ける力
  };
  DebugData debugData; // デバッグ用のOutPortから出力するためのデータ. AutoStabilizer内の制御処理では使われることは無い. そのため、モード遷移や初期化等の処理にはあまり注意を払わなくて良い

public:
  void init(const cnoid::BodyPtr& robot_){
    maxTorque.resize(robot_->numJoints(), std::numeric_limits<double>::max());
    jointLimitTables.resize(robot_->numJoints());
    softMaxTorque.resize(robot_->numJoints(), std::numeric_limits<double>::max());
    jointControllable.resize(robot_->numJoints(), true);

    robot = std::make_shared<Object>(robot_);

    refRobotRaw = robot_->clone();
    refRobotRaw->calcForwardKinematics(); refRobotRaw->calcCenterOfMass();
    actRobotRaw = robot_->clone();
    actRobotRaw->calcForwardKinematics(); actRobotRaw->calcCenterOfMass();
  }

  void onExecute(double dt){ // onExecuteで毎周期呼ばれる. 内部の補間器をdtすすめる
    robot->onExecute();
    for(int i=0;i<objects.size();i++) objects[i]->onExecute();
    for(std::unordered_map<std::string, std::shared_ptr<Contact> >::iterator it=contacts.begin();it!=contacts.end();it++) it.second->onExecute();
    for(std::unordered_map<std::string, std::shared_ptr<Attention> >::iterator it=attentions.begin();it!=attentions.end();it++) it.second->onExecute();

    for(int i=0;i<softMaxTorque.size();i++) softMaxTorque[i].reset(softMaxTorque[i].getGoal());
  }

  // startAutoBalancer時に呼ばれる
  void onStartAutoBalancer(){
    robot->onStartAutoBalancer();
    for(int i=0;i<objects.size();i++) objects[i]->onStartAutoBalancer();
    for(std::unordered_map<std::string, std::shared_ptr<Contact> >::iterator it=contacts.begin();it!=contacts.end();it++) it.second->onStartAutoBalancer();
    for(std::unordered_map<std::string, std::shared_ptr<Attention> >::iterator it=attentions.begin();it!=attentions.end();it++) it.second->onStartAutoBalancer();

  }

  // startStabilizer時に呼ばれる
  void onStartStabilizer(){
    robot->onStartStabilizer();
    for(int i=0;i<objects.size();i++) objects[i]->onStartStabilizer();
    for(std::unordered_map<std::string, std::shared_ptr<Contact> >::iterator it=contacts.begin();it!=contacts.end();it++) it.second->onStartStabilizer();
    for(std::unordered_map<std::string, std::shared_ptr<Attention> >::iterator it=attentions.begin();it!=attentions.end();it++) it.second->onStartStabilizer();

  }

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

// for debug
inline std::ostream &operator<<(std::ostream &os, const std::vector<cnoid::Vector3>& polygon){
  for(int j=0;j<polygon.size();j++){
    os << polygon[j].format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", " [", "]"));
  }
  return os;
}

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



#endif

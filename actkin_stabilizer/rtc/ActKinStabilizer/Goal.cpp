#include "Goal.h"
#include "MathUtil.h"
#include <unordered_set>
#include <eigen_rtm_conversions/eigen_rtm_conversions.h>
#include <rtm_data_tools/rtm_data_tools.h>

namespace actkin_stabilizer{
  void Goal::init(const State& state){
    return;
  }

  void Goal::updateFromIdl(const State& state, const actkin_stabilizer_msgs::RefStateIdl& m_refState){
    if(m_refState.feasibility != actkin_stabilizer_msgs::INFEASIBLE){
      this->updateFromIdl(state, m_refState.refEEPose);
      this->updateFromIdl(state, m_refState.refVRP);
      this->updateFromIdl(state, m_refState.refq);
      this->updateFromIdl(state, m_refState.refContact);
    }
  }

  void Goal::updateFromIdl(const State& state, const actkin_stabilizer_msgs::RefEESequence& m_refEEPose){
    std::unordered_map<std::string, std::shared_ptr<RefEE> > nextEEGoals;
    for(int i=0;i<m_refEEPose.length();i++){
      std::string name = std::string(m_refEEPose[i].name);
      std::shared_ptr<RefEE> eeGoal;
      if(this->eeGoals.find(name) != this->eeGoals.end()){
        eeGoal = this->eeGoals[name];
      }else{
        eeGoal = std::make_shared<RefEE>();
      }

      eeGoal->name = name;

      if(state.linkNameMap.find(std::string(m_refEEPose[i].link)) == state.linkNameMap.end()){
        std::cerr << __FUNCTION__ << m_refEEPose[i].link << " not found" << std::endl;
        continue;
      }
      eeGoal->link = state.linkNameMap.find(std::string(m_refEEPose[i].link))->second;

      if(!rtm_data_tools::isAllFinite(m_refEEPose[i].localPose)){
        std::cerr << __FUNCTION__ << "local_pose not finite" << std::endl;
        continue;
      }
      eigen_rtm_conversions::poseRTMToEigen(m_refEEPose[i].localPose, eeGoal->localPose);

      if(state.linkNameMap.find(std::string(m_refEEPose[i].frameId)) == state.linkNameMap.end()){
        std::cerr << __FUNCTION__ << m_refEEPose[i].frameId << " not found" << std::endl;
        continue;
      }
      eeGoal->frameLink = state.linkNameMap.find(std::string(m_refEEPose[i].frameId))->second;

      if(!rtm_data_tools::isAllFinite(m_refEEPose[i].framePose)){
        std::cerr << __FUNCTION__ << "local_pose not finite" << std::endl;
        continue;
      }
      eigen_rtm_conversions::poseRTMToEigen(m_refEEPose[i].framePose, eeGoal->framePose);

      for(int j=0;j<6;j++) eeGoal->freeAxis[j] = m_refEEPose[i].freeAxis[j];

      eeGoal->priority = m_refEEPose[i].priority;

      cnoid::Isometry3 p; // global frame
      cnoid::Vector6 dp; // global frame. p origin
      cnoid::Vector6 ddp; // global frame. p origin
      if(eeGoal->pose.size()>0){
        eeGoal->pose[0].value(p,dp,ddp);
      }else{ // 今回始めて現れたgoal
        const cnoid::Isometry3 parent_pose = (eeGoal->link) ? eeGoal->link->T() : cnoid::Isometry3::Identity(); // world frame
        p = parent_pose * eeGoal->localPose; // world frame
        dp = cnoid::Vector6::Zero(); // world frame. p origin
        if(eeGoal->link){
          dp.head<3>() += eeGoal->link->v();
          dp.head<3>() += eeGoal->link->w().cross(parent_pose.linear() * eeGoal->localPose.translation());
          dp.tail<3>() += eeGoal->link->w();
        }

        ddp.setZero();
      }

      cnoid::Vector6 f; // global frame. p origin
      cnoid::Vector6 df; // global frame. p origin
      cnoid::Vector6 ddf; // global frame. p origin
      if(eeGoal->wrench.size()>0){
        eeGoal->wrench[0].value(f,df,ddf);
      }else{ // 今回始めて現れたgoal
        f.setZero();
        df.setZero();
        ddf.setZero();
      }

      eeGoal->pose.clear();
      eeGoal->wrench.clear();
      for(int j=0;j<m_refEEPose[i].trajectory.length();j++){
        double time;
        cnoid::Isometry3 goal_p; // global frame
        cnoid::Vector6 goal_dp; // global frame. p origin
        cnoid::Vector6 goal_f; // global frame. p origin
        if(!std::isfinite(m_refEEPose[i].trajectory[j].time)){
          std::cerr << __FUNCTION__ << " time is not finite!" << std::endl;
          continue;
        }
        time = std::max(0.0, m_refEEPose[i].trajectory[j].time);
        if(!rtm_data_tools::isAllFinite(m_refEEPose[i].trajectory[j].pose)){
          std::cerr << __FUNCTION__ << "pose not finite" << std::endl;
          continue;
        }
        eigen_rtm_conversions::poseRTMToEigen(m_refEEPose[i].trajectory[j].pose, goal_p);
        if(!rtm_data_tools::isAllFinite(m_refEEPose[i].trajectory[j].velocity)){
          std::cerr << __FUNCTION__ << "velocity not finite" << std::endl;
          continue;
        }
        eigen_rtm_conversions::velocityRTMToEigen(m_refEEPose[i].trajectory[j].velocity, goal_dp);
        if(!rtm_data_tools::isAllFinite(m_refEEPose[i].trajectory[j].wrench)){
          std::cerr << __FUNCTION__ << "wrench not finite" << std::endl;
          continue;
        }
        eigen_rtm_conversions::vectorRTMToEigen(m_refEEPose[i].trajectory[j].wrench, goal_f);

        eeGoal->pose.emplace_back(p,dp,ddp,cpp_filters::HOFFARBIB);
        eeGoal->pose.back().setGoal(goal_p, goal_dp, time);
        eeGoal->wrench.emplace_back(f,df,ddf,cpp_filters::HOFFARBIB);
        eeGoal->wrench.back().setGoal(goal_f, time);

        p = goal_p;
        dp = goal_dp;
        ddp = cnoid::Vector6::Zero();
        f = goal_f;
        df = cnoid::Vector6::Zero();
        ddf = cnoid::Vector6::Zero();
      }

      if(eeGoal->pose.size() == 0 || eeGoal->wrench.size() == 0) {
        std::cerr << __FUNCTION__ << "trajectory is empty" << std::endl;
        continue;
      }

      {
        if(!eeGoal->positionConstraint){
          eeGoal->positionConstraint = std::make_shared<aik_constraint::PositionConstraint>();
        }
        eeGoal->positionConstraint->A_link() = eeGoal->link;
        eeGoal->positionConstraint->A_localpos() = eeGoal->localPose;
        eeGoal->positionConstraint->B_link() = nullptr;
        for(int k=0;k<6;k++){
          eeGoal->positionConstraint->weight()[k] = eeGoal->freeAxis[k] ? 0.0 : 1.0;
        }
        eeGoal->positionConstraint->eval_link() = nullptr;
        eeGoal->positionConstraint->maxAccByPosError() = 5 * cnoid::Vector6::Ones();
        eeGoal->positionConstraint->maxAccByVelError() = 10 * cnoid::Vector6::Ones();
        eeGoal->positionConstraint->maxAcc() = 15 * cnoid::Vector6::Ones();
        eeGoal->positionConstraint->pgain() << this->Kp, this->Kp, this->Kp, this->Kr, this->Kr, this->Kr;
        eeGoal->positionConstraint->dgain() << this->Dp, this->Dp, this->Dp, this->Dr, this->Dr, this->Dr;
      }


      nextEEGoals[name] = eeGoal;
    }

    std::swap(this->eeGoals, nextEEGoals);
  }

  void Goal::updateFromIdl(const State& state, const actkin_stabilizer_msgs::RefVRPIdl& m_refVRP){
    std::vector<std::shared_ptr<RefVRP> > nextVRPGoals;

    for(int i=0;i<1;i++){
      std::shared_ptr<RefVRP> vrpGoal;
      if(this->vrpGoals.size() > 0){
        vrpGoal = this->vrpGoals[0];
      }else{
        vrpGoal = std::make_shared<RefVRP>();
      }

      if(!std::isfinite(m_refVRP.omega)){
        std::cerr << __FUNCTION__ << " omega is not finite" << std::endl;
        continue;
      }
      if(m_refVRP.omega <= 0.0) continue;
      vrpGoal->omega = m_refVRP.omega;

      cnoid::Vector3 p; // world frame
      cnoid::Vector3 dp; // world frame
      cnoid::Vector3 ddp; // world frame
      if(vrpGoal->vrp.size()>0){
        vrpGoal->vrp[0].value(p,dp,ddp);
      }else{ // 今回始めて現れたvrp
        p = state.robot->centerOfMass();
        dp = state.cogVel;//state.cogVel.value();
        ddp.setZero();
      }

      vrpGoal->vrp.clear();
      for(int j=0;j<m_refVRP.trajectory.length();j++){
        double time;
        cnoid::Vector3 goal_p; // global frame
        if(!std::isfinite(m_refVRP.trajectory[j].time)){
          std::cerr << __FUNCTION__ << " time is not finite!" << std::endl;
          continue;
        }
        time = std::max(0.0, m_refVRP.trajectory[j].time);
        if(!rtm_data_tools::isAllFinite(m_refVRP.trajectory[j].point)){
          std::cerr << __FUNCTION__ << "point not finite" << std::endl;
          continue;
        }
        eigen_rtm_conversions::pointRTMToEigen(m_refVRP.trajectory[j].point, goal_p);

        vrpGoal->vrp.emplace_back(p,dp,ddp,cpp_filters::LINEAR);
        vrpGoal->vrp.back().setGoal(goal_p, time);

        p = goal_p;
        dp = cnoid::Vector3::Zero();
        ddp = cnoid::Vector3::Zero();
      }

      if(vrpGoal->vrp.size() == 0) {
        std::cerr << __FUNCTION__ << "trajectory is empty" << std::endl;
        continue;
      }

      if(!vrpGoal->comConstraint){
        vrpGoal->comConstraint = std::make_shared<aik_constraint::COMConstraint>();
      }
      vrpGoal->comConstraint->A_robot() = state.robot;
      vrpGoal->comConstraint->weight() << 1.0, 1.0, 0.3;
      vrpGoal->comConstraint->pgain().setZero();
      vrpGoal->comConstraint->dgain().setZero();

      if(!vrpGoal->force) {
        vrpGoal->force = std::make_shared<aik_constraint::Force>();
        vrpGoal->force->F() = cnoid::VectorX::Zero(1);
        vrpGoal->force->S().resize(6,1);
        for(int j=0;j<3;j++) vrpGoal->force->S().insert(j,0) = 1.0 / this->forceRatio;
        vrpGoal->force->B_link() = state.robot->rootLink();
      }

      if(!vrpGoal->forceConstraint){
        vrpGoal->forceConstraint = std::make_shared<aik_constraint::ForceConstraint>();
        vrpGoal->forceConstraint->force() = vrpGoal->force;
        vrpGoal->forceConstraint->dl() = cnoid::VectorX::Zero(1);
        vrpGoal->forceConstraint->du() = cnoid::VectorX::Zero(1);
        vrpGoal->forceConstraint->C().resize(1,1);
        vrpGoal->forceConstraint->C().insert(0,0) = 1.0;
      }

      if(!vrpGoal->force2) {
        vrpGoal->force2 = std::make_shared<aik_constraint::Force>();
        vrpGoal->force2->F() = cnoid::VectorX::Zero(3);
        vrpGoal->force2->S().resize(6,3);
        for(int j=0;j<3;j++) vrpGoal->force2->S().insert(j,j) = 1.0 / this->forceRatio;
        vrpGoal->force2->B_link() = state.robot->rootLink();
      }

      if(!vrpGoal->force2Constraint1){
        vrpGoal->force2Constraint1 = std::make_shared<aik_constraint::ForceConstraint>();
        vrpGoal->force2Constraint1->force() = vrpGoal->force2;
        vrpGoal->force2Constraint1->dl() = cnoid::VectorX::Zero(3);
        vrpGoal->force2Constraint1->du() = cnoid::VectorX::Zero(3);
        vrpGoal->force2Constraint1->C().resize(3,3);
        for(int j=0;j<3;j++) vrpGoal->force2Constraint1->C().insert(j,j) = 1.0;
      }

      if(!vrpGoal->force2Constraint2){
        vrpGoal->force2Constraint2 = std::make_shared<aik_constraint::ForceConstraint>();
        vrpGoal->force2Constraint2->force() = vrpGoal->force2;
        vrpGoal->force2Constraint2->dl() = cnoid::VectorX::Zero(2);
        vrpGoal->force2Constraint2->du() = cnoid::VectorX::Zero(2);
        vrpGoal->force2Constraint2->C().resize(2,3);
        vrpGoal->force2Constraint2->C().insert(0,0) = 1.0;
        vrpGoal->force2Constraint2->C().insert(1,1) = 1.0;
      }

      if(!vrpGoal->angularMomentumConstraint){
        vrpGoal->angularMomentumConstraint = std::make_shared<aik_constraint::AngularMomentumConstraint>();
      }
      vrpGoal->angularMomentumConstraint->robot() = state.robot;
      vrpGoal->angularMomentumConstraint->weight() << 0.1, 0.1, 0.1;

      nextVRPGoals.push_back(vrpGoal);
    }

    std::swap(this->vrpGoals, nextVRPGoals);
  }

  void Goal::updateFromIdl(const State& state, const actkin_stabilizer_msgs::RefqIdl& m_refq) {
    std::vector<std::shared_ptr<Refq> > nextqGoals;

    for(int i=0;i<1;i++){
      std::shared_ptr<Refq> qGoal;
      if(this->qGoals.size() > 0){
        qGoal = this->qGoals[0];
      }else{
        qGoal = std::make_shared<Refq>();
      }

      cnoid::VectorX q; // world frame
      cnoid::VectorX dq; // world frame
      cnoid::VectorX ddq; // world frame
      if(qGoal->q.size()>0){
        qGoal->q[0].value(q,dq,ddq);
      }else{ // 今回始めて現れたq
        q = cnoid::VectorX(state.robot->numJoints());
        for(int j=0;j<state.robot->numJoints();j++) q[j] = state.robot->joint(j)->q();
        dq = cnoid::VectorX::Zero(state.robot->numJoints());
        ddq = cnoid::VectorX::Zero(state.robot->numJoints());
      }

      qGoal->q.clear();
      for(int j=0;j<m_refq.trajectory.length();j++){
        double time;
        cnoid::VectorX goal_q;
        cnoid::VectorX goal_dq;
        if(!std::isfinite(m_refq.trajectory[j].time)){
          std::cerr << __FUNCTION__ << " time is not finite!" << std::endl;
          continue;
        }
        time = std::max(0.0, m_refq.trajectory[j].time);
        if(!rtm_data_tools::isAllFinite(m_refq.trajectory[j].q)){
          std::cerr << __FUNCTION__ << "q not finite" << std::endl;
          continue;
        }
        if(m_refq.trajectory[j].q.length() != state.robot->numJoints()){
          std::cerr << __FUNCTION__ << "q dimension mismatch" << std::endl;
          continue;
        }
        eigen_rtm_conversions::vectorRTMToEigen(m_refq.trajectory[j].q, goal_q);
        if(!rtm_data_tools::isAllFinite(m_refq.trajectory[j].dq)){
          std::cerr << __FUNCTION__ << "dq not finite" << std::endl;
          continue;
        }
        if(m_refq.trajectory[j].dq.length() != state.robot->numJoints()){
          std::cerr << __FUNCTION__ << "dq dimension mismatch" << std::endl;
          continue;
        }
        eigen_rtm_conversions::vectorRTMToEigen(m_refq.trajectory[j].dq, goal_dq);

        qGoal->q.emplace_back(q,dq,ddq,cpp_filters::HOFFARBIB);
        qGoal->q.back().setGoal(goal_q, goal_dq, time);

        q = goal_q;
        dq = goal_dq;
        ddq = cnoid::Vector3::Zero();
      }

      if(qGoal->q.size() == 0) {
        std::cerr << __FUNCTION__ << "trajectory is empty" << std::endl;
        continue;
      }

      qGoal->jointAngleConstraints.resize(state.robot->numJoints());
      for(int j=0;j<state.robot->numJoints();j++){
        if(!qGoal->jointAngleConstraints[j]) qGoal->jointAngleConstraints[j] = std::make_shared<aik_constraint::JointAngleConstraint>();
        qGoal->jointAngleConstraints[j]->joint() = state.robot->joint(j);
        // qGoal->jointAngleConstraints[j]->pgain() = 400.0;
        // qGoal->jointAngleConstraints[j]->dgain() = 50.0;
        // qGoal->jointAngleConstraints[j]->maxAccByPosError() = 3.0;
        // qGoal->jointAngleConstraints[j]->maxAccByVelError() = 10.0;
        // qGoal->jointAngleConstraints[j]->maxAcc() = 15.0;
      }

      nextqGoals.push_back(qGoal);
    }

    std::swap(this->qGoals, nextqGoals);

  }

  void Goal::updateFromIdl(const State& state, const actkin_stabilizer_msgs::RefContactSequence& m_refContact){
    std::unordered_map<std::string, std::shared_ptr<RefContact> > nextContactGoals;

    for(int i=0;i<m_refContact.length();i++){
      std::string name = std::string(m_refContact[i].name);
      std::shared_ptr<RefContact> contactGoal;
      if(this->contactGoals.find(name) != this->contactGoals.end()){
        contactGoal = this->contactGoals[name];
      }else{
        contactGoal = std::make_shared<RefContact>();
      }

      contactGoal->name = name;

      if(state.linkNameMap.find(std::string(m_refContact[i].link1)) == state.linkNameMap.end()){
        std::cerr << __FUNCTION__ << m_refContact[i].link1 << " not found" << std::endl;
        continue;
      }
      contactGoal->link1 = state.linkNameMap.find(std::string(m_refContact[i].link1))->second;

      if(!rtm_data_tools::isAllFinite(m_refContact[i].localPose1)){
        std::cerr << __FUNCTION__ << "localPose1 not finite" << std::endl;
        continue;
      }
      eigen_rtm_conversions::poseRTMToEigen(m_refContact[i].localPose1, contactGoal->localPose1);

      if(state.linkNameMap.find(std::string(m_refContact[i].link2)) == state.linkNameMap.end()){
        std::cerr << __FUNCTION__ << m_refContact[i].link2 << " not found" << std::endl;
        continue;
      }
      contactGoal->link2 = state.linkNameMap.find(std::string(m_refContact[i].link2))->second;

      for(int j=0;j<6;j++) contactGoal->freeAxis[j] = m_refContact[i].freeAxis[j];

      if(!rtm_data_tools::isAllFinite(m_refContact[i].region.C) ||
         !rtm_data_tools::isAllFinite(m_refContact[i].region.ld) ||
         !rtm_data_tools::isAllFinite(m_refContact[i].region.ud)){
        std::cerr << __FUNCTION__ << "region not finite" << std::endl;
        continue;
      }
      eigen_rtm_conversions::matrixRTMToEigen(m_refContact[i].region.C, contactGoal->region.C);
      eigen_rtm_conversions::vectorRTMToEigen(m_refContact[i].region.ld, contactGoal->region.ld);
      eigen_rtm_conversions::vectorRTMToEigen(m_refContact[i].region.ud, contactGoal->region.ud);
      if(contactGoal->region.C.cols() != 3 ||
         contactGoal->region.C.rows() != contactGoal->region.ld.rows() ||
         contactGoal->region.C.rows() != contactGoal->region.ud.rows()){
        std::cerr << __FUNCTION__ << "region dimension mismatch" << std::endl;
        continue;
      }

      if(std::isfinite(m_refContact[i].muTrans)){
        contactGoal->muTrans = std::max(0.0, m_refContact[i].muTrans);
      }else{
        std::cerr << __FUNCTION__ << "muTrans not finite" << std::endl;
        continue;
      }
      if(std::isfinite(m_refContact[i].muRot)){
        contactGoal->muRot = std::max(0.0, m_refContact[i].muRot);
      }else{
        std::cerr << __FUNCTION__ << "muRot not finite" << std::endl;
        continue;
      }
      if(std::isfinite(m_refContact[i].maxFz)){
        contactGoal->maxFz = std::max(0.0, m_refContact[i].maxFz);
      }else{
        std::cerr << __FUNCTION__ << "maxFz not finite" << std::endl;
        continue;
      }
      if(std::isfinite(m_refContact[i].minFz)){
        contactGoal->minFz = std::max(0.0, m_refContact[i].minFz);
      }else{
        std::cerr << __FUNCTION__ << "minFz not finite" << std::endl;
        continue;
      }
      {
        std::vector<Eigen::Vector2d> surface(m_refContact[i].surface.length());
        for(int j=0; j<m_refContact[i].surface.length();j++){
          if(rtm_data_tools::isAllFinite(m_refContact[i].surface[j])){
            eigen_rtm_conversions::vectorRTMToEigen(m_refContact[i].surface[j], surface[j]);
          }else{
            std::cerr << __FUNCTION__ << "surface not finite" << std::endl;
            surface.clear();
            break;
          }
        }
        mathutil::calcConvexHull(surface,contactGoal->surface);
        if(surface.size() == 0) {
          std::cerr << __FUNCTION__ << "surface is empty" << std::endl;
          continue;
        }
      }

      if(!contactGoal->force ||
         contactGoal->force->A_link() != contactGoal->link1 ||
         contactGoal->force->B_link() != contactGoal->link2) {
        contactGoal->force = std::make_shared<aik_constraint::Force>();
        aik_constraint::Force::setFACE(contactGoal->force);
        contactGoal->force->S() /= this->forceRatio;
      }
      contactGoal->force->A_link() = contactGoal->link1;
      contactGoal->force->A_localpos() = contactGoal->localPose1;
      contactGoal->force->B_link() = contactGoal->link2;

      if(!contactGoal->forceConstraint){
        contactGoal->forceConstraint = std::make_shared<aik_constraint::ForceConstraint>();
      }
      contactGoal->forceConstraint->force() = contactGoal->force;
      contactGoal->forceConstraint->dl().resize(0);
      contactGoal->forceConstraint->du().resize(0);
      contactGoal->forceConstraint->C().resize(0,6);

      if(!contactGoal->forceReductionConstraint){
        contactGoal->forceReductionConstraint = std::make_shared<aik_constraint::ForceConstraint>();
      }
      contactGoal->forceReductionConstraint->force() = contactGoal->force;
      contactGoal->forceReductionConstraint->dl() = Eigen::VectorXd::Zero(5) * this->forceRatio;
      contactGoal->forceReductionConstraint->C().resize(5,6);
      contactGoal->forceReductionConstraint->du() = Eigen::VectorXd::Zero(5) * this->forceRatio;
      contactGoal->forceReductionConstraint->C().insert(0,0) = 1e0;
      contactGoal->forceReductionConstraint->C().insert(1,1) = 1e0;
      contactGoal->forceReductionConstraint->C().insert(2,3) = 1e0;
      contactGoal->forceReductionConstraint->C().insert(3,4) = 1e0;
      contactGoal->forceReductionConstraint->C().insert(4,5) = 1e0;

      if(!contactGoal->positionConstraint) {
        contactGoal->positionConstraint = std::make_shared<aik_constraint::PositionConstraint>();
      }
      contactGoal->positionConstraint->A_link() = contactGoal->link1;
      contactGoal->positionConstraint->A_localpos() = contactGoal->localPose1;
      contactGoal->positionConstraint->B_link() = contactGoal->link2;
      // 相対速度0
      contactGoal->positionConstraint->pgain().setZero();
      contactGoal->positionConstraint->dgain() << this->contactDp, this->contactDp, this->contactDp, this->contactDr, this->contactDr, this->contactDr;
      //contactGoal->positionConstraint->dgain().setZero();
      contactGoal->positionConstraint->ref_acc().setZero();

      nextContactGoals[name] = contactGoal;
    }

    std::swap(this->contactGoals, nextContactGoals);

  }

  void Goal::onStartStabilizer(){
    this->eeGoals.clear();
    this->vrpGoals.clear();
    this->qGoals.clear();
    this->contactGoals.clear();
    return;
  }

  void Goal::interpolate(double dt){
    for(std::unordered_map<std::string, std::shared_ptr<RefEE> >::iterator it = this->eeGoals.begin(); it!=this->eeGoals.end(); it++){
      if(it->second->pose[0].isEmpty() && it->second->pose.size() > 1){
        it->second->pose.erase(it->second->pose.begin());
      }
      it->second->pose[0].interpolate(dt);
      if(it->second->wrench[0].isEmpty() && it->second->wrench.size() > 1){
        it->second->wrench.erase(it->second->wrench.begin());
      }
      it->second->wrench[0].interpolate(dt);
    }

    for(int i=0;i<this->vrpGoals.size();i++){
      if(this->vrpGoals[i]->vrp[0].isEmpty() && this->vrpGoals[i]->vrp.size() > 1){
        this->vrpGoals[i]->vrp.erase(this->vrpGoals[i]->vrp.begin());
      }
      this->vrpGoals[i]->vrp[0].interpolate(dt);
    }

    for(int i=0;i<this->qGoals.size();i++){
      if(this->qGoals[i]->q[0].isEmpty() && this->qGoals[i]->q.size() > 1){
        this->qGoals[i]->q.erase(this->qGoals[i]->q.begin());
      }
      this->qGoals[i]->q[0].interpolate(dt);
    }

  }
};



// static function
// void State::calcActiveObjectsContacts(const std::shared_ptr<Object>& robot, const std::unordered_map<std::string, std::shared_ptr<Object> >& objects, const std::unordered_map<std::string, std::shared_ptr<Contact> >& contacts,
//                                           std::vector<std::shared_ptr<Object> >& activeObjects, std::vector<std::shared_ptr<Contact> >& activeContacts){
//   activeObjects.clear();
//   activeContacts.clear();
//   std::unordered_set<cnoid::BodyPtr> activeBodies; // robotとcontactを介してつながっているobjects.
//   std::vector<std::shared_ptr<Contact> > tmpContacts; tmpContacts.reserve(contacts.size());
//   std::vector<std::shared_ptr<Contact> > nextTmpContacts; nextTmpContacts.reserve(contacts.size());
//   for(std::unordered_map<std::string, std::shared_ptr<Contact> >::const_iterator it = contacts.begin(); it != contacts.end(); it++) nextTmpContacts.push_back(it->second);
//   while(tmpContacts.size() != nextTmpContacts.size()){
//     tmpContacts = nextTmpContacts;
//     nextTmpContacts.clear();
//     for(int i=0;i<tmpContacts.size();i++){
//       if(tmpContacts[i]->link1 &&
//          ((tmpContacts[i]->link1->body() == robot->body) || (activeBodies.find(tmpContacts[i]->link1->body()) != activeBodies.end()))){
//         if(tmpContacts[i]->link2 && tmpContacts[i]->link2->body() != robot->body) activeBodies.insert(tmpContacts[i]->link2->body());
//         activeContacts.push_back(tmpContacts[i]);
//       }else if(tmpContacts[i]->link2 &&
//                ((tmpContacts[i]->link2->body() == robot->body) || (activeBodies.find(tmpContacts[i]->link2->body()) != activeBodies.end()))){
//         if(tmpContacts[i]->link1 && tmpContacts[i]->link1->body() != robot->body) activeBodies.insert(tmpContacts[i]->link1->body());
//         activeContacts.push_back(tmpContacts[i]);
//       }else{
//         nextTmpContacts.push_back(tmpContacts[i]);
//       }
//     }
//   }
//   for(std::unordered_map<std::string, std::shared_ptr<Object> >::const_iterator it = objects.begin(); it != objects.end(); it++){
//     if(activeBodies.find(it->second->body) != activeBodies.end()) {
//       activeObjects.push_back(it->second);
//     }else{
//       it->second->onStartAutoBalancer(); // 以後速度が上書きされなくなるので、0にresetしておく
//     }
//   }
//   for(std::unordered_map<std::string, std::shared_ptr<Contact> >::const_iterator it = contacts.begin(); it != contacts.end(); it++){
//     it->second->onStartAutoBalancer(); // actToGenFrameConverterの制約が変化するので、goActualしておく
//   }
// }

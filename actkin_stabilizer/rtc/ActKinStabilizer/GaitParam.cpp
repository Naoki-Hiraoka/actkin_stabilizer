#include "GaitParam.h"
#include <unordered_set>
#include <eigen_rtm_conversions/eigen_rtm_conversions.h>
#include "RTMUtil.h"

bool Contact::initializeFromIdl(const std::shared_ptr<Object>& robot, const std::unordered_map<std::string, std::shared_ptr<Object> >& objects, const actkin_stabilizer::ContactParamIdl& idl){
  // idlから初期化

  this->name = idl.name;
  this->stateId = idl.stateId + 1; // 1 増える
  if(std::string(idl.obj1) == ""){
    if(std::string(idl.link1) == ""){
      this->link1 = nullptr; // world
    }else{
      cnoid::LinkPtr link = robot->body->link(std::string(idl.link1));
      if(link) {
        this->link1 = link; // robotのlink
      }else{
        std::cerr << "contact link is not found! " << idl.link1 << std::endl;
        return false;
      }
    }
  }else{
    std::unordered_map<std::string, std::shared_ptr<Object> >::const_iterator object_it = objects.find(std::string(idl.obj1));
    if(object_it != objects.end()){
      cnoid::LinkPtr link = object_it->second->body->link(std::string(idl.link1));
      if(link) {
        this->link1 = link; // objectのlink
      }else{
        std::cerr << "contact link is not found! " << idl.link1 << std::endl;
        return false;
      }
    }else{
      std::cerr << "contact object is not found! " << idl.obj1 << std::endl;
      return false;
    }
  }
  if(!rtmutil::isAllFinite(idl.localPose1)){
    std::cerr << "contact localPose1 is not finite! " << std::endl;
    return false;
  }
  eigen_rtm_conversions::poseRTMToEigen(idl.localPose1, this->localPose1);
  if(std::string(idl.obj2) == ""){
    if(std::string(idl.link2) == ""){
      this->link2 = nullptr; // world
    }else{
      cnoid::LinkPtr link = robot->body->link(std::string(idl.link2));
      if(link) {
        this->link2 = link; // robotのlink
      }else{
        std::cerr << "contact link is not found! " << idl.link2 << std::endl;
        return false;
      }
    }
  }else{
    std::unordered_map<std::string, std::shared_ptr<Object> >::const_iterator object_it = objects.find(std::string(idl.obj2));
    if(object_it != objects.end()){
      cnoid::LinkPtr link = object_it->second->body->link(std::string(idl.link2));
      if(link) {
        this->link2 = link; // objectのlink
      }else{
        std::cerr << "contact link is not found! " << idl.link2 << std::endl;
        return false;
      }
    }else{
      std::cerr << "contact object is not found! " << idl.obj2 << std::endl;
      return false;
    }
  }
  for(int a=0;a<idl.axis.length();a++) this->axis[a] = idl.axis[a] ? 1.0 : 0.0;
  int num_axis = (this->axis.array() == 1.0).count();
  this->S = Eigen::SparseMatrix<double,Eigen::RowMajor>(num_axis,6);
  num_axis = 0;
  for(int a=0;a<idl.axis.length();a++) {
    if(idl.axis[a]) {
      this->S.insert(num_axis,a) = 1.0;
      num_axis++;
    }
  }
  if(!rtmutil::isAllFinite(idl.C)){ std::cerr << "contact C is not finite! " << std::endl; return false; }
  for(int row = 0; row < idl.C.length(); row++){
    if(idl.C[row].length() != this->S.rows()) { std::cerr << "contact C dimension mismatch! " << std::endl; return false; }
  }
  this->C = Eigen::SparseMatrix<double,Eigen::RowMajor>(idl.C.length(),this->S.rows());
  for(int row = 0; row < idl.C.length(); row++){
    for(int col = 0; col < idl.C[row].length(); col++){
      if(idl.C[row][col] != 0.0) {
        this->C.insert(row,col) = idl.C[row][col];
      }
    }
  }
  if(!rtmutil::isAllFinite(idl.ld) || !rtmutil::isAllFinite(idl.ud)){ std::cerr << "contact ld/ud is not finite! " << std::endl; return false; }
  if(idl.ld.length() != idl.C.length() || idl.ud.length() != idl.C.length()){ std::cerr << "contact ld/ud dimension mismatch! " << idl.ld.length() << " " << idl.ud.length() << std::endl; return false; }
  eigen_rtm_conversions::vectorRTMToEigen(idl.ld, this->ld);
  eigen_rtm_conversions::vectorRTMToEigen(idl.ud, this->ud);
  if(!rtmutil::isAllFinite(idl.w)){ std::cerr << "contact w is not finite! " << std::endl; return false; }
  if(idl.w.length() != this->S.rows()){ std::cerr << "contact w dimension mismatch! " << idl.w.length() << std::endl; return false; }
  eigen_rtm_conversions::vectorRTMToEigen(idl.w, this->w);
  return true;
}

void Contact::copyToIdl(actkin_stabilizer::ContactParamIdl& idl){
  idl.name = this->name.c_str();
  idl.stateId = this->stateId;
  idl.remove = false;
  idl.obj1 = this->link1 ? this->link1->body()->name().c_str() : "";
  idl.link1 = this->link1 ? this->link1->name().c_str() : "";
  eigen_rtm_conversions::poseEigenToRTM(this->localPose1, idl.localPose1);
  idl.obj2 = this->link2 ? this->link2->body()->name().c_str() : "";
  idl.link2 = this->link2 ? this->link2->name().c_str() : "";
  idl.axis.length(6); for(int i=0;i<6;i++) idl.axis[i] = this->axis[i] == 1.0;
  eigen_rtm_conversions::matrixEigenToRTM(this->C, idl.C);
  eigen_rtm_conversions::vectorEigenToRTM(this->ld, idl.ld);
  eigen_rtm_conversions::vectorEigenToRTM(this->ud, idl.ud);
  eigen_rtm_conversions::vectorEigenToRTM(this->w, idl.w);
}

bool Attention::initializeFromIdl(const std::shared_ptr<Object>& robot, const std::unordered_map<std::string, std::shared_ptr<Object> >& objects, const actkin_stabilizer::AttentionParamIdl& idl){
  // idlから初期化
  this->name = idl.name;
  this->stateId = idl.stateId + 1; // 1 増える
  if(std::string(idl.obj1) == ""){
    if(std::string(idl.link1) == ""){
      this->cog1 = nullptr;
      this->link1 = nullptr; // world
    }else if(std::string(idl.link1) == "CM"){
      if(std::string(idl.obj2) == "" && std::string(idl.link2) == ""){
        this->cog1 = robot->body;
        this->link1 = nullptr; // robotの重心
        if(idl.horizon>0.0) this->localPose2.setInterpolationMode(cpp_filters::LINEAR);
      }else{
        std::cerr << "attention link should be world! " << std::endl;
        return false;
      }
    }else{
      cnoid::LinkPtr link = robot->body->link(std::string(idl.link1));
      if(link) {
        this->cog1 = nullptr;
        this->link1 = link; // robotのlink
      }else{
        std::cerr << "attention link is not found! " << idl.link1 << std::endl;
        return false;
      }
    }
  }else{
    std::unordered_map<std::string, std::shared_ptr<Object> >::const_iterator object_it = objects.find(std::string(idl.obj1));
    if(object_it != objects.end()){
      if(std::string(idl.link1) == "CM"){
        if(std::string(idl.obj2) == "" && std::string(idl.link2) == ""){
          this->cog1 = object_it->second->body;
          this->link1 = nullptr; // objectの重心
          if(idl.horizon>0.0) this->localPose2.setInterpolationMode(cpp_filters::LINEAR);
        }else{
          std::cerr << "attention link should be world! " << std::endl;
          return false;
        }
      }else{
        cnoid::LinkPtr link = object_it->second->body->link(std::string(idl.link1));
        if(link) {
          this->cog1 = nullptr;
          this->link1 = link; // objectのlink
        }else{
          std::cerr << "attention link is not found! " << idl.link1 << std::endl;
          return false;
        }
      }
    }else{
      std::cerr << "attention object is not found! " << idl.obj1 << std::endl;
      return false;
    }
  }
  if(!rtmutil::isAllFinite(idl.localPose1)){
    std::cerr << "attention localPose1 is not finite! " << std::endl;
    return false;
  }
  cnoid::Position localPose1;
  eigen_rtm_conversions::poseRTMToEigen(idl.localPose1, localPose1);
  this->localPose1.reset(localPose1);
  if(std::string(idl.obj2) == ""){
    if(std::string(idl.link2) == ""){
      this->cog2 = nullptr;
      this->link2 = nullptr; // world
    }else if(std::string(idl.link2) == "CM"){
      if(std::string(idl.obj1) == "" && std::string(idl.link1) == ""){
        this->cog2 = robot->body;
        this->link2 = nullptr; // robotの重心
        if(idl.horizon>0.0) this->localPose1.setInterpolationMode(cpp_filters::LINEAR);
      }else{
        std::cerr << "attention link should be world! " << std::endl;
        return false;
      }
    }else{
      cnoid::LinkPtr link = robot->body->link(std::string(idl.link2));
      if(link) {
        this->cog2 = nullptr;
        this->link2 = link; // robotのlink
      }else{
        std::cerr << "attention link is not found! " << idl.link2 << std::endl;
        return false;
      }
    }
  }else{
    std::unordered_map<std::string, std::shared_ptr<Object> >::const_iterator object_it = objects.find(std::string(idl.obj2));
    if(object_it != objects.end()){
      if(std::string(idl.link2) == "CM"){
        if(std::string(idl.obj1) == "" && std::string(idl.link1) == ""){
          this->cog2 = object_it->second->body;
          this->link2 = nullptr; // objectの重心
          if(idl.horizon>0.0) this->localPose1.setInterpolationMode(cpp_filters::LINEAR);
        }else{
          std::cerr << "attention link should be world! " << std::endl;
          return false;
        }
      }else{
        cnoid::LinkPtr link = object_it->second->body->link(std::string(idl.link2));
        if(link) {
          this->cog2 = nullptr;
          this->link2 = link; // objectのlink
        }else{
          std::cerr << "attention link is not found! " << idl.link2 << std::endl;
          return false;
        }
      }
    }else{
      std::cerr << "attention object is not found! " << idl.obj2 << std::endl;
      return false;
    }
  }
  if(!rtmutil::isAllFinite(idl.localPose2)){ std::cerr << "attention localPose2 is not finite! " << std::endl; return false; }
  cnoid::Position localPose2;
  eigen_rtm_conversions::poseRTMToEigen(idl.localPose2, localPose2);
  this->localPose2.reset(localPose2);
  if(!rtmutil::isAllFinite(idl.refWrench)){ std::cerr << "attention refWrench is not finite! " << std::endl; return false; }
  if(idl.refWrench.length() != 6){ std::cerr << "attention refWrench dimension mismatch! " << idl.refWrench.length() << std::endl; return false; }
  cnoid::Vector6 refWrench;
  eigen_rtm_conversions::vectorRTMToEigen(idl.refWrench,refWrench);
  this->refWrench.reset(refWrench);
  if(!rtmutil::isAllFinite(idl.C)){ std::cerr << "attention C is not finite! " << std::endl; return false; }
  for(int row = 0; row < idl.C.length(); row++){
    if(idl.C[row].length() != 6) { std::cerr << "attention C dimension mismatch! " << std::endl; return false; }
  }
  this->C = Eigen::SparseMatrix<double,Eigen::RowMajor>(idl.C.length(),6);
  for(int row = 0; row < idl.C.length(); row++){
    for(int col = 0; col < idl.C[row].length(); col++){
      if(idl.C[row][col] != 0.0) {
        this->C.insert(row,col) = idl.C[row][col];
      }
    }
  }
  if(!rtmutil::isAllFinite(idl.ld) || !rtmutil::isAllFinite(idl.ud)){ std::cerr << "attention ld/ud is not finite! " << std::endl; return false; }
  if(idl.ld.length() != idl.C.length() || idl.ud.length() != idl.C.length()){ std::cerr << "attention ld/ud dimension mismatch! " << idl.ld.length() << " " << idl.ud.length() << std::endl; return false; }
  cnoid::VectorX ld_; eigen_rtm_conversions::vectorRTMToEigen(idl.ld, ld_);
  ld.reset(ld_, cnoid::VectorX::Zero(6), cnoid::VectorX::Zero(6));
  cnoid::VectorX ud_; eigen_rtm_conversions::vectorRTMToEigen(idl.ud, ud_);
  ud.reset(ud_, cnoid::VectorX::Zero(6), cnoid::VectorX::Zero(6));
  if(!rtmutil::isAllFinite(idl.Kp) || !rtmutil::isAllFinite(idl.Dp)){ std::cerr << "attention Kp/Dp is not finite! " << std::endl; return false; }
  if(idl.Kp.length() != idl.C.length() || idl.Dp.length() != idl.C.length()){ std::cerr << "attention Kp/Dp dimension mismatch! " << idl.Kp.length() << " " << idl.Dp.length() << std::endl; return false; }
  eigen_rtm_conversions::vectorRTMToEigen(idl.Kp, this->Kp);
  eigen_rtm_conversions::vectorRTMToEigen(idl.Dp, this->Dp);
  if(!rtmutil::isAllFinite(idl.limitp)){ std::cerr << "attention limitp is not finite! " << std::endl; return false; }
  if(idl.limitp.length() != idl.C.length()){ std::cerr << "attention limitp dimension mismatch! " << idl.limitp.length() << std::endl; return false; }
  eigen_rtm_conversions::vectorRTMToEigen(idl.limitp, this->limitp);
  if(!rtmutil::isAllFinite(idl.weightp)){ std::cerr << "attention weightp is not finite! " << std::endl; return false; }
  if(idl.weightp.length() != idl.C.length()){ std::cerr << "attention weightp dimension mismatch! " << idl.weightp.length() << std::endl; return false; }
  eigen_rtm_conversions::vectorRTMToEigen(idl.weightp, this->weightp);
  this->priority = idl.priority;
  this->horizon = idl.horizon;
  if(!rtmutil::isAllFinite(idl.Kw) || !rtmutil::isAllFinite(idl.Dw)){ std::cerr << "attention Kw/Dw is not finite! " << std::endl; return false; }
  if(idl.Kw.length() != idl.C.length() || idl.Dw.length() != idl.C.length()){ std::cerr << "attention Kw/Dw dimension mismatch! " << idl.Kw.length() << " " << idl.Dw.length() << std::endl; return false; }
  eigen_rtm_conversions::vectorRTMToEigen(idl.Kw, this->Kw);
  eigen_rtm_conversions::vectorRTMToEigen(idl.Dw, this->Dw);
  if(!rtmutil::isAllFinite(idl.limitw)){ std::cerr << "attention limitw is not finite! " << std::endl; return false; }
  if(idl.limitw.length() != idl.C.length()){ std::cerr << "attention limitw dimension mismatch! " << idl.limitw.length() << std::endl; return false; }
  eigen_rtm_conversions::vectorRTMToEigen(idl.limitw, this->limitw);

  return true;
}

bool Attention::updateFromIdl(const actkin_stabilizer::AttentionDataIdl& idl){
  // idlから更新
  if(idl.localPose1.length()>0){
    cnoid::Position pose;
    cnoid::Vector6 velocity = cnoid::Vector6::Zero(); // pose local frame
    cnoid::Vector6 acceleration = cnoid::Vector6::Zero(); // pose local frame
    eigen_rtm_conversions::poseRTMToEigen(idl.localPose1[0].pose, pose);
    if(idl.localPose1[0].velocity.length() == 6) {
      eigen_rtm_conversions::vectorRTMToEigen(idl.localPose1[0].velocity, velocity); // world frame
      velocity.head<3>() = (pose.linear().transpose() * velocity.head<3>()).eval(); // pose local frame
      velocity.tail<3>() = (pose.linear().transpose() * velocity.tail<3>()).eval(); // pose local frame
    }
    if(idl.localPose1[0].acceleration.length() == 6) {
      eigen_rtm_conversions::vectorRTMToEigen(idl.localPose1[0].acceleration, acceleration); // world frame
      acceleration.head<3>() = (pose.linear().transpose() * acceleration.head<3>()).eval(); // pose local frame
      acceleration.tail<3>() = (pose.linear().transpose() * acceleration.tail<3>()).eval(); // pose local frame
    }
    if(idl.localPose1[0].time >= 0.0) this->localPose1.setGoal(pose, velocity, acceleration, idl.localPose1[0].time);
  }
  if(idl.localPose2.length()>0){
    cnoid::Position pose;
    cnoid::Vector6 velocity = cnoid::Vector6::Zero(); // pose local frame
    cnoid::Vector6 acceleration = cnoid::Vector6::Zero(); // pose local frame
    eigen_rtm_conversions::poseRTMToEigen(idl.localPose2[0].pose, pose);
    if(idl.localPose2[0].velocity.length() == 6) {
      eigen_rtm_conversions::vectorRTMToEigen(idl.localPose2[0].velocity, velocity); // world frame
      velocity.head<3>() = (pose.linear().transpose() * velocity.head<3>()).eval(); // pose local frame
      velocity.tail<3>() = (pose.linear().transpose() * velocity.tail<3>()).eval(); // pose local frame
    }
    if(idl.localPose2[0].acceleration.length() == 6) {
      eigen_rtm_conversions::vectorRTMToEigen(idl.localPose2[0].acceleration, acceleration); // world frame
      acceleration.head<3>() = (pose.linear().transpose() * acceleration.head<3>()).eval(); // pose local frame
      acceleration.tail<3>() = (pose.linear().transpose() * acceleration.tail<3>()).eval(); // pose local frame
    }
    if(idl.localPose2[0].time >= 0.0) this->localPose2.setGoal(pose, velocity, acceleration, idl.localPose2[0].time);
  }
  if(idl.ld.length()>0){
    cnoid::VectorX vector = cnoid::Vector6::Zero(); // pose local frame
    eigen_rtm_conversions::vectorRTMToEigen(idl.ld[0].vector, vector);
    if(vector.size() == this->ld.value().size()) this->ld.setGoal(vector, idl.ld[0].time);
  }
  if(idl.ud.length()>0){
    cnoid::VectorX vector = cnoid::Vector6::Zero(); // pose local frame
    eigen_rtm_conversions::vectorRTMToEigen(idl.ud[0].vector, vector);
    if(vector.size() == this->ud.value().size()) this->ud.setGoal(vector, idl.ud[0].time);
  }
  if(idl.refWrench.length()>0){
    cnoid::VectorX vector = cnoid::Vector6::Zero(); // pose local frame
    eigen_rtm_conversions::vectorRTMToEigen(idl.refWrench[0].vector, vector);
    if(vector.size() == 6) this->refWrench.setGoal(vector, idl.refWrench[0].time);
  }
  return true;
}

void Attention::copyToIdl(actkin_stabilizer::AttentionParamIdl& idl){
  idl.name = this->name.c_str();
  idl.stateId = this->stateId;
  idl.remove = false;
  idl.obj1 = this->cog1 ? this->cog1->name().c_str() : this->link1 ? this->link1->body()->name().c_str() : "";
  idl.link1 = this->cog1 ? "CM" : this->link1 ? this->link1->body()->name().c_str() : "";
  eigen_rtm_conversions::poseEigenToRTM(this->localPose1.value(), idl.localPose1);
  idl.obj2 = this->cog2 ? this->cog2->name().c_str() : this->link2 ? this->link2->body()->name().c_str() : "";
  idl.link2 = this->cog2 ? "CM" : this->link2 ? this->link2->body()->name().c_str() : "";
  eigen_rtm_conversions::poseEigenToRTM(this->localPose2.value(), idl.localPose2);
  eigen_rtm_conversions::vectorEigenToRTM(this->refWrench.value(), idl.refWrench);
  eigen_rtm_conversions::matrixEigenToRTM(this->C, idl.C);
  eigen_rtm_conversions::vectorEigenToRTM(this->ld.value(), idl.ld);
  eigen_rtm_conversions::vectorEigenToRTM(this->ud.value(), idl.ud);
  eigen_rtm_conversions::vectorEigenToRTM(this->Kp, idl.Kp);
  eigen_rtm_conversions::vectorEigenToRTM(this->Dp, idl.Dp);
  eigen_rtm_conversions::vectorEigenToRTM(this->limitp, idl.limitp);
  eigen_rtm_conversions::vectorEigenToRTM(this->weightp, idl.weightp);
  idl.priority = this->priority;
  idl.horizon = this->horizon;
  eigen_rtm_conversions::vectorEigenToRTM(this->Kw, idl.Kw);
  eigen_rtm_conversions::vectorEigenToRTM(this->Dw, idl.Dw);
  eigen_rtm_conversions::vectorEigenToRTM(this->limitw, idl.limitw);
}

// static function
void GaitParam::calcActiveObjectsContacts(const std::shared_ptr<Object>& robot, const std::unordered_map<std::string, std::shared_ptr<Object> >& objects, const std::unordered_map<std::string, std::shared_ptr<Contact> >& contacts,
                                          std::vector<std::shared_ptr<Object> >& activeObjects, std::vector<std::shared_ptr<Contact> >& activeContacts){
  activeObjects.clear();
  activeContacts.clear();
  std::unordered_set<cnoid::BodyPtr> activeBodies; // robotとcontactを介してつながっているobjects.
  std::vector<std::shared_ptr<Contact> > tmpContacts; tmpContacts.reserve(contacts.size());
  std::vector<std::shared_ptr<Contact> > nextTmpContacts; nextTmpContacts.reserve(contacts.size());
  for(std::unordered_map<std::string, std::shared_ptr<Contact> >::const_iterator it = contacts.begin(); it != contacts.end(); it++) nextTmpContacts.push_back(it->second);
  while(tmpContacts.size() != nextTmpContacts.size()){
    tmpContacts = nextTmpContacts;
    nextTmpContacts.clear();
    for(int i=0;i<tmpContacts.size();i++){
      if(tmpContacts[i]->link1 &&
         ((tmpContacts[i]->link1->body() == robot->body) || (activeBodies.find(tmpContacts[i]->link1->body()) != activeBodies.end()))){
        if(tmpContacts[i]->link2 && tmpContacts[i]->link2->body() != robot->body) activeBodies.insert(tmpContacts[i]->link2->body());
        activeContacts.push_back(tmpContacts[i]);
      }else if(tmpContacts[i]->link2 &&
               ((tmpContacts[i]->link2->body() == robot->body) || (activeBodies.find(tmpContacts[i]->link2->body()) != activeBodies.end()))){
        if(tmpContacts[i]->link1 && tmpContacts[i]->link1->body() != robot->body) activeBodies.insert(tmpContacts[i]->link1->body());
        activeContacts.push_back(tmpContacts[i]);
      }else{
        nextTmpContacts.push_back(tmpContacts[i]);
      }
    }
  }
  for(std::unordered_map<std::string, std::shared_ptr<Object> >::const_iterator it = objects.begin(); it != objects.end(); it++){
    if(activeBodies.find(it->second->body) != activeBodies.end()) {
      activeObjects.push_back(it->second);
    }else{
      it->second->onStartAutoBalancer(); // 以後速度が上書きされなくなるので、0にresetしておく
    }
  }
  for(std::unordered_map<std::string, std::shared_ptr<Contact> >::const_iterator it = contacts.begin(); it != contacts.end(); it++){
    it->second->onStartAutoBalancer(); // actToGenFrameConverterの制約が変化するので、goActualしておく
  }
}

// static function
void GaitParam::calcPrioritizedAttentions(const std::unordered_map<std::string, std::shared_ptr<Attention> >& attentions,
                                          std::vector<std::vector<std::shared_ptr<Attention> > >& prioritizedAttentions){
  prioritizedAttentions.clear();
  std::vector<std::shared_ptr<Attention> > tmpAttentions; tmpAttentions.reserve(attentions.size());
  std::vector<std::shared_ptr<Attention> > nextTmpAttentions; nextTmpAttentions.reserve(attentions.size());
  for(std::unordered_map<std::string, std::shared_ptr<Attention> >::const_iterator it = attentions.begin(); it != attentions.end(); it++) tmpAttentions.push_back(it->second);
  while(tmpAttentions.size() > 0){
    long maxPriority = std::numeric_limits<long>::min();
    for(int i=0;i<tmpAttentions.size();i++) {
      if(tmpAttentions[i]->priority > maxPriority) maxPriority = tmpAttentions[i]->priority;
    }
    prioritizedAttentions.emplace_back();
    nextTmpAttentions.clear();
    for(int i=0;i<tmpAttentions.size();i++) {
      if(tmpAttentions[i]->priority == maxPriority) prioritizedAttentions.back().push_back(tmpAttentions[i]);
      else nextTmpAttentions.push_back(tmpAttentions[i]);
    }
    tmpAttentions = nextTmpAttentions;
  }
}

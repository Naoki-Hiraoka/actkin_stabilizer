// -*-C++-*-
#ifndef ActKinStabilizerSERVICESVC_IMPL_H
#define ActKinStabilizerSERVICESVC_IMPL_H

#include "actkin_stabilizer/idl/ActKinStabilizerService.hh"

class ActKinStabilizer;

class ActKinStabilizerService_impl
  : public virtual POA_actkin_stabilizer::ActKinStabilizerService,
    public virtual PortableServer::RefCountServantBase
{
public:
  ActKinStabilizerService_impl();// 実装は.cppファイルの方に書かないと、registerProvider時にSegmentation Faultになる
  ~ActKinStabilizerService_impl();
  CORBA::Boolean startAutoBalancer();
  CORBA::Boolean stopAutoBalancer();
  CORBA::Boolean startStabilizer();
  CORBA::Boolean stopStabilizer();
  CORBA::Boolean setPrimitiveState(const actkin_stabilizer::PrimitiveStateIdl& command);
  CORBA::Boolean getPrimitiveState(actkin_stabilizer::PrimitiveStateIdl_out command);
  CORBA::Boolean resetPrimitiveState(const actkin_stabilizer::PrimitiveStateIdl& command);
  CORBA::Boolean goActual();
  CORBA::Boolean loadObject(const char *name, const char *file);
  CORBA::Boolean unloadObject(const char *name);
  CORBA::Boolean setObjectState(const actkin_stabilizer::ObjectStateIdl& obj);
  CORBA::Boolean setObjectStates(const actkin_stabilizer::ObjectStateIdlSeq& objs);
  CORBA::Boolean getObjectStates(actkin_stabilizer::ObjectStateIdlSeq_out objs);

  CORBA::Boolean setActKinStabilizerParam(const actkin_stabilizer::ActKinStabilizerService::ActKinStabilizerParam& i_param);
  CORBA::Boolean getActKinStabilizerParam(actkin_stabilizer::ActKinStabilizerService::ActKinStabilizerParam_out i_param);
  //
  //
  void setComp(ActKinStabilizer *i_comp);
private:
  ActKinStabilizer *comp_;
};

#endif

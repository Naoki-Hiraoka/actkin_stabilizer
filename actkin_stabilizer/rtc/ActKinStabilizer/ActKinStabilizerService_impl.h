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
  CORBA::Boolean goPos( CORBA::Double x,  CORBA::Double y,  CORBA::Double th);
  CORBA::Boolean goVelocity( CORBA::Double vx,  CORBA::Double vy,  CORBA::Double vth);
  CORBA::Boolean goStop();
  CORBA::Boolean jumpTo( CORBA::Double x,  CORBA::Double y,  CORBA::Double z,  CORBA::Double ts,  CORBA::Double tf);
  CORBA::Boolean setFootSteps(const actkin_stabilizer::ActKinStabilizerService::FootstepSequence& fs);
  CORBA::Boolean setFootStepsWithParam(const actkin_stabilizer::ActKinStabilizerService::FootstepSequence& fs, const actkin_stabilizer::ActKinStabilizerService::StepParamSequence& spss);
  void waitFootSteps();
  CORBA::Boolean startAutoBalancer();
  CORBA::Boolean stopAutoBalancer();
  CORBA::Boolean startStabilizer(void);
  CORBA::Boolean stopStabilizer(void);
  CORBA::Boolean setActKinStabilizerParam(const actkin_stabilizer::ActKinStabilizerService::ActKinStabilizerParam& i_param);
  CORBA::Boolean getActKinStabilizerParam(actkin_stabilizer::ActKinStabilizerService::ActKinStabilizerParam_out i_param);
  CORBA::Boolean getFootStepState(actkin_stabilizer::ActKinStabilizerService::FootStepState_out i_param);
  CORBA::Boolean releaseEmergencyStop();
  CORBA::Boolean startImpedanceController(const char *i_name_);
  CORBA::Boolean stopImpedanceController(const char *i_name_);
  CORBA::Boolean startWholeBodyMasterSlave();
  CORBA::Boolean stopWholeBodyMasterSlave();
  //
  //
  void setComp(ActKinStabilizer *i_comp);
private:
  ActKinStabilizer *comp_;
};

#endif

#include "ActKinStabilizerService_impl.h"
#include "ActKinStabilizer.h"

ActKinStabilizerService_impl::ActKinStabilizerService_impl()
{
}

ActKinStabilizerService_impl::~ActKinStabilizerService_impl()
{
}

CORBA::Boolean ActKinStabilizerService_impl::goPos( CORBA::Double x,  CORBA::Double y,  CORBA::Double th)
{
  return this->comp_->goPos(x, y, th);
};

CORBA::Boolean ActKinStabilizerService_impl::goVelocity( CORBA::Double vx,  CORBA::Double vy,  CORBA::Double vth)
{
  return this->comp_->goVelocity(vx, vy, vth);
};

CORBA::Boolean ActKinStabilizerService_impl::goStop()
{
  return this->comp_->goStop();
};

CORBA::Boolean ActKinStabilizerService_impl::jumpTo( CORBA::Double x,  CORBA::Double y,  CORBA::Double z,  CORBA::Double ts,  CORBA::Double tf)
{
  return this->comp_->jumpTo(x, y, z, ts, tf);
};

CORBA::Boolean ActKinStabilizerService_impl::setFootSteps(const actkin_stabilizer::ActKinStabilizerService::FootstepSequence& fs)
{
  return this->comp_->setFootSteps(fs);
}

CORBA::Boolean ActKinStabilizerService_impl::setFootStepsWithParam(const actkin_stabilizer::ActKinStabilizerService::FootstepSequence& fs, const actkin_stabilizer::ActKinStabilizerService::StepParamSequence& spss)
{
  return this->comp_->setFootStepsWithParam(fs, spss);
}

void ActKinStabilizerService_impl::waitFootSteps()
{
  return this->comp_->waitFootSteps();
};

CORBA::Boolean ActKinStabilizerService_impl::startAutoBalancer()
{
  return this->comp_->startAutoBalancer();
};

CORBA::Boolean ActKinStabilizerService_impl::stopAutoBalancer()
{
  return this->comp_->stopAutoBalancer();
};

CORBA::Boolean ActKinStabilizerService_impl::startStabilizer(void)
{
  return this->comp_->startStabilizer();
}

CORBA::Boolean ActKinStabilizerService_impl::stopStabilizer(void)
{
  return this->comp_->stopStabilizer();
}

CORBA::Boolean ActKinStabilizerService_impl::setActKinStabilizerParam(const actkin_stabilizer::ActKinStabilizerService::ActKinStabilizerParam& i_param)
{
  return this->comp_->setActKinStabilizerParam(i_param);
};

CORBA::Boolean ActKinStabilizerService_impl::getActKinStabilizerParam(actkin_stabilizer::ActKinStabilizerService::ActKinStabilizerParam_out i_param)
{
  i_param = new actkin_stabilizer::ActKinStabilizerService::ActKinStabilizerParam();
  return this->comp_->getActKinStabilizerParam(*i_param);
};

CORBA::Boolean ActKinStabilizerService_impl::getFootStepState(actkin_stabilizer::ActKinStabilizerService::FootStepState_out i_param)
{
  i_param = new actkin_stabilizer::ActKinStabilizerService::FootStepState();
  return this->comp_->getFootStepState(*i_param);
};

CORBA::Boolean ActKinStabilizerService_impl::releaseEmergencyStop()
{
    return this->comp_->releaseEmergencyStop();
};

CORBA::Boolean ActKinStabilizerService_impl::startImpedanceController(const char *i_name_)
{
  return this->comp_->startImpedanceController(i_name_);
};

CORBA::Boolean ActKinStabilizerService_impl::stopImpedanceController(const char *i_name_)
{
  return this->comp_->stopImpedanceController(i_name_);
};

CORBA::Boolean ActKinStabilizerService_impl::startWholeBodyMasterSlave(void)
{
  return this->comp_->startWholeBodyMasterSlave();
}

CORBA::Boolean ActKinStabilizerService_impl::stopWholeBodyMasterSlave(void)
{
  return this->comp_->stopWholeBodyMasterSlave();
}

void ActKinStabilizerService_impl::setComp(ActKinStabilizer *i_comp)
{
  this->comp_ = i_comp;
}

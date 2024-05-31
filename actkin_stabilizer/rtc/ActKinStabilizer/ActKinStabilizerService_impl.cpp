#include "ActKinStabilizerService_impl.h"
#include "ActKinStabilizer.h"

ActKinStabilizerService_impl::ActKinStabilizerService_impl()
{
}

ActKinStabilizerService_impl::~ActKinStabilizerService_impl()
{
}

CORBA::Boolean ActKinStabilizerService_impl::startStabilizer()
{
  return this->comp_->startStabilizer();
}

CORBA::Boolean ActKinStabilizerService_impl::stopStabilizer()
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

void ActKinStabilizerService_impl::setComp(ActKinStabilizer *i_comp)
{
  this->comp_ = i_comp;
}

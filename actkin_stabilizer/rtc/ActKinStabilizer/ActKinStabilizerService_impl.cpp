#include "ActKinStabilizerService_impl.h"
#include "ActKinStabilizer.h"

ActKinStabilizerService_impl::ActKinStabilizerService_impl()
{
}

ActKinStabilizerService_impl::~ActKinStabilizerService_impl()
{
}

CORBA::Boolean ActKinStabilizerService_impl::startAutoBalancer()
{
  return this->comp_->startAutoBalancer();
};

CORBA::Boolean ActKinStabilizerService_impl::stopAutoBalancer()
{
  return this->comp_->stopAutoBalancer();
};

CORBA::Boolean ActKinStabilizerService_impl::startStabilizer()
{
  return this->comp_->startStabilizer();
}

CORBA::Boolean ActKinStabilizerService_impl::stopStabilizer()
{
  return this->comp_->stopStabilizer();
}


CORBA::Boolean ActKinStabilizerService_impl::setPrimitiveState(const actkin_stabilizer::PrimitiveStateIdl& command)
{
  return this->comp_->setPrimitiveState(command);
}

CORBA::Boolean ActKinStabilizerService_impl::getPrimitiveState(actkin_stabilizer::PrimitiveStateIdl_out command)
{
  command = new actkin_stabilizer::PrimitiveStateIdl();
  return this->comp_->getPrimitiveState(*command);
};

CORBA::Boolean ActKinStabilizerService_impl::resetPrimitiveState(const actkin_stabilizer::PrimitiveStateIdl& command)
{
  return this->comp_->resetPrimitiveState(command);
}

CORBA::Boolean ActKinStabilizerService_impl::goActual()
{
  return this->comp_->goActual();
}

CORBA::Boolean ActKinStabilizerService_impl::loadObject(const char *name, const char *file)
{
  return this->comp_->loadObject(name, file);
};

CORBA::Boolean ActKinStabilizerService_impl::unloadObject(const char *name)
{
  return this->comp_->unloadObject(name);
};

CORBA::Boolean ActKinStabilizerService_impl::setObjectState(const actkin_stabilizer::ObjectStateIdl& obj)
{
  return this->comp_->setObjectState(obj);
}

CORBA::Boolean ActKinStabilizerService_impl::setObjectStates(const actkin_stabilizer::ObjectStateIdlSeq& objs)
{
  return this->comp_->setObjectStates(objs);
}

CORBA::Boolean ActKinStabilizerService_impl::getObjectStates(actkin_stabilizer::ObjectStateIdlSeq_out objs)
{
  objs = new actkin_stabilizer::ObjectStateIdlSeq();
  return this->comp_->getObjectStates(*objs);
};

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

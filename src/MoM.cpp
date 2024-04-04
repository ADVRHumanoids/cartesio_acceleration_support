#include "MoM.h"
#include <boost/make_shared.hpp>

using namespace XBot::Cartesian::acceleration;

OpenSotMomAdapter::OpenSotMomAdapter(TaskDescription::Ptr task,
                                     Context::ConstPtr context):
    OpenSotTaskAdapter(task, context),
    _old_lambda(0.0)
{
    _ci_mom = std::dynamic_pointer_cast<AngularMomentum>(task);

    if(!_ci_mom) throw std::runtime_error("Provided task description "
                                           "does not have expected type 'AngularMomentumTask'");

}

TaskPtr OpenSotMomAdapter::constructTask()
{
    _opensot_mom = SotUtils::make_shared<MomSoT>(const_cast<ModelInterface&>(*_model),
                                              _vars.getVariable("qddot"));


    return _opensot_mom;
}

bool OpenSotMomAdapter::initialize(const OpenSoT::OptvarHelper& vars)
{
    bool ret = OpenSotTaskAdapter::initialize(vars);
    if(!ret) return false;


    /* Cartesian task specific parameters */
    _old_lambda = _opensot_mom->getLambda();


    /* Register observer */

    auto this_shared_ptr = std::dynamic_pointer_cast<OpenSotMomAdapter>(shared_from_this());
    _ci_mom->registerObserver(this_shared_ptr);

    return true;
}

void OpenSotMomAdapter::update(double time, double period)
{
    const double dt = _ctx->params()->getControlPeriod();

    _opensot_mom->setLambda(_ci_mom->getLambda()/dt);


    Eigen::Vector3d amom = _ci_mom->getReference();
    _opensot_mom->setReference(amom);
}

bool OpenSotMomAdapter::onBaseLinkChanged()
{
    return false;
}

bool OpenSotMomAdapter::onControlModeChanged()
{
    auto ctrl = _ci_cartesian->getControlMode();

    if(ctrl == ControlType::Position)
    {
        _opensot_mom->setLambda(_old_lambda);
    }
    else if(ctrl == ControlType::Velocity)
    {
        _old_lambda = _opensot_mom->getLambda();
        _opensot_mom->setLambda(0.0);
    }

    return true;
}

OpenSoT::OptvarHelper::VariableVector OpenSotMomAdapter::getRequiredVariables() const
{
    return {{"qddot", _model->getNv()}};
}

CARTESIO_REGISTER_OPENSOT_TASK_PLUGIN(OpenSotMomAdapter, AngularMomentum)


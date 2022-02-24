#include "Cartesian.h"
#include <boost/make_shared.hpp>

using namespace XBot::Cartesian::acceleration;

OpenSotCartesianAdapter::OpenSotCartesianAdapter(TaskDescription::Ptr task,
                                                 Context::ConstPtr context):
    OpenSotTaskAdapter(task, context),
    _old_lambda(0.0)
{
    _ci_cart = std::dynamic_pointer_cast<CartesianTask>(task);

    if(!_ci_cart) throw std::runtime_error("Provided task description "
                                           "does not have expected type 'CartesianTask'");


}

TaskPtr OpenSotCartesianAdapter::constructTask()
{
    Eigen::VectorXd q;
    _model->getJointPosition(q);

    _opensot_cart = SotUtils::make_shared<CartesianSoT>(_ci_cart->getName(),
                                                     const_cast<ModelInterface&>(*_model),
                                                     _ci_cart->getDistalLink(),
                                                     _ci_cart->getBaseLink(),
                                                     _vars.getVariable("qddot"));


    return _opensot_cart;
}

bool OpenSotCartesianAdapter::initialize(const OpenSoT::OptvarHelper& vars)
{
    bool ret = OpenSotTaskAdapter::initialize(vars);
    if(!ret) return false;


    /* Cartesian task specific parameters */

    _old_lambda = _opensot_cart->getLambda();
    if(_ci_cart->isSubtaskLocal())
    {
        throw std::invalid_argument("Local subtask not supported");
    }

    if(_ci_cart->getLambda2() >= 0)
    {
        const double dt = _ctx->params()->getControlPeriod();
        _opensot_cart->setLambda(_ci_cart->getLambda()/dt/dt,
                                 _ci_cart->getLambda2()/dt);
    }

    /* Register observer */

    auto this_shared_ptr = std::dynamic_pointer_cast<OpenSotCartesianAdapter>(shared_from_this());
    _ci_cart->registerObserver(this_shared_ptr);

    return true;
}

void OpenSotCartesianAdapter::update(double time, double period)
{
    const double dt = _ctx->params()->getControlPeriod();

    if(_ci_cart->getLambda2() >= 0)
    {
        _opensot_cart->setLambda(_ci_cart->getLambda()/dt/dt,
                                 _ci_cart->getLambda2()/dt);
    }
    else
    {
        _opensot_cart->setLambda(_ci_cart->getLambda()/dt/dt);

    }

    /* Update reference */
    Eigen::Affine3d Tref;
    Eigen::Vector6d vref;
    Eigen::Vector6d aref;

    _ci_cart->getPoseReference(Tref, &vref, &aref);
    _opensot_cart->setReference(Tref, vref, aref);
}

bool OpenSotCartesianAdapter::onBaseLinkChanged()
{
    return _opensot_cart->setBaseLink(_ci_cart->getBaseLink());
}

bool OpenSotCartesianAdapter::onControlModeChanged()
{
    auto ctrl = _ci_cart->getControlMode();

    if(ctrl == ControlType::Position)
    {
        _opensot_cart->setLambda(_old_lambda);
    }
    else if(ctrl == ControlType::Velocity)
    {
        _old_lambda = _opensot_cart->getLambda();
        _opensot_cart->setLambda(0.0);
    }

    return true;
}

OpenSoT::OptvarHelper::VariableVector OpenSotCartesianAdapter::getRequiredVariables() const
{
    return {{"qddot", _model->getJointNum()}};
}

CARTESIO_REGISTER_OPENSOT_TASK_PLUGIN(OpenSotCartesianAdapter, Cartesian)



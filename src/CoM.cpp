#include "CoM.h"
#include <boost/make_shared.hpp>

using namespace XBot::Cartesian::acceleration;

OpenSotComAdapter::OpenSotComAdapter(TaskDescription::Ptr task,
                                     Context::ConstPtr context):
    OpenSotTaskAdapter(task, context),
    _old_lambda(0.0)
{
    _ci_com = std::dynamic_pointer_cast<ComTask>(task);

    if(!_ci_com) throw std::runtime_error("Provided task description "
                                           "does not have expected type 'ComTask'");

}

TaskPtr OpenSotComAdapter::constructTask()
{
    Eigen::VectorXd q;
    _model->getJointPosition(q);

    _opensot_com = SotUtils::make_shared<ComSoT>(const_cast<ModelInterface&>(*_model),
                                              _vars.getVariable("qddot"));


    return _opensot_com;
}

bool OpenSotComAdapter::initialize(const OpenSoT::OptvarHelper& vars)
{
    bool ret = OpenSotTaskAdapter::initialize(vars);
    if(!ret) return false;


    /* Cartesian task specific parameters */

    _old_lambda = _opensot_com->getLambda();
    if(_ci_com->isSubtaskLocal())
    {
        throw std::invalid_argument("Local subtask not supported");
    }

    if(_ci_com->getLambda2() >= 0)
    {
        const double dt = _ctx->params()->getControlPeriod();
        _opensot_com->setLambda(_ci_com->getLambda()/dt/dt,
                                 _ci_com->getLambda2()/dt);
    }

    /* Register observer */

    auto this_shared_ptr = std::dynamic_pointer_cast<OpenSotComAdapter>(shared_from_this());
    _ci_com->registerObserver(this_shared_ptr);

    return true;
}

void OpenSotComAdapter::update(double time, double period)
{
    const double dt = _ctx->params()->getControlPeriod();

    if(_ci_com->getLambda2() >= 0)
    {
        _opensot_com->setLambda(_ci_com->getLambda()/dt/dt,
                                 _ci_com->getLambda2()/dt);
    }
    else
    {
        _opensot_com->setLambda(_ci_com->getLambda()/dt/dt);

    }

    /* Update reference */
    Eigen::Affine3d Tref;
    Eigen::Vector6d vref;
    _ci_com->getPoseReference(Tref, &vref);
    _opensot_com->setReference(Tref.translation(), vref.segment(0,3));
}

bool OpenSotComAdapter::onBaseLinkChanged()
{
    return false;
}

bool OpenSotComAdapter::onControlModeChanged()
{
    auto ctrl = _ci_com->getControlMode();

    if(ctrl == ControlType::Position)
    {
        _opensot_com->setLambda(_old_lambda);
    }
    else if(ctrl == ControlType::Velocity)
    {
        _old_lambda = _opensot_com->getLambda();
        _opensot_com->setLambda(0.0);
    }

    return true;
}

OpenSoT::OptvarHelper::VariableVector OpenSotComAdapter::getRequiredVariables() const
{
    return {{"qddot", _model->getJointNum()}};
}

CARTESIO_REGISTER_OPENSOT_TASK_PLUGIN(OpenSotComAdapter, Com)

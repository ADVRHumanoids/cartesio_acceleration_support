#include "Interaction.h"

#include <boost/make_shared.hpp>

using namespace XBot::Cartesian::acceleration;

OpenSotInteractionAdapter::OpenSotInteractionAdapter(TaskDescription::Ptr task,
                                                     Context::ConstPtr context):
    OpenSotTaskAdapter(task, context)
{
    _ci_cart = std::dynamic_pointer_cast<InteractionTask>(task);

    if(!_ci_cart) throw std::runtime_error("Provided task description "
                                           "does not have expected type 'InteractionTask'");


}

TaskPtr OpenSotInteractionAdapter::constructTask()
{
    Eigen::VectorXd q;
    _model->getJointPosition(q);

    _opensot_cart = std::make_shared<CartesianSoT>(_ci_cart->getName(),
                                                     const_cast<ModelInterface&>(*_model),
                                                     _ci_cart->getDistalLink(),
                                                     _ci_cart->getBaseLink(),
                                                     _vars.getVariable("qddot"));

    _opensot_cart->setGainType(OpenSoT::tasks::acceleration::Force);

    return _opensot_cart;
}

bool OpenSotInteractionAdapter::initialize(const OpenSoT::OptvarHelper& vars)
{
    bool ret = OpenSotTaskAdapter::initialize(vars);
    if(!ret) return false;


    /* Cartesian task specific parameters */
    if(_ci_cart->isSubtaskLocal())
    {
        throw std::invalid_argument("Local subtask not supported");
    }

    /* Register observer */
    auto this_shared_ptr = std::dynamic_pointer_cast<OpenSotInteractionAdapter>(shared_from_this());
    _ci_cart->registerObserver(this_shared_ptr);

    return true;
}

void OpenSotInteractionAdapter::update(double time, double period)
{
    /* Update reference */
    Eigen::Affine3d Tref;
    Eigen::Vector6d vref;
    _ci_cart->getPoseReference(Tref, &vref);

    _opensot_cart->setLambda(1, 1);

    _opensot_cart->setReference(Tref, vref);

    _opensot_cart->setKp(_ci_cart->getStiffness());
    _opensot_cart->setKd(_ci_cart->getDamping());
    _opensot_cart->setVirtualForce(_ci_cart->getForceReference());
}

bool OpenSotInteractionAdapter::onBaseLinkChanged()
{
    return _opensot_cart->setBaseLink(_ci_cart->getBaseLink());
}

bool OpenSotInteractionAdapter::onControlModeChanged()
{
    return false;
}

OpenSoT::OptvarHelper::VariableVector OpenSotInteractionAdapter::getRequiredVariables() const
{
    return {{"qddot", _model->getJointNum()}};
}

CARTESIO_REGISTER_OPENSOT_TASK_PLUGIN(OpenSotInteractionAdapter, Interaction)



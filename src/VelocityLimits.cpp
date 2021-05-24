#include "VelocityLimits.h"

using namespace XBot::Cartesian::acceleration;


OpenSotVelocityLimitsAdapter::OpenSotVelocityLimitsAdapter(ConstraintDescription::Ptr constr,
                                                     Context::ConstPtr context):
    OpenSotConstraintAdapter (constr, context)
{
    _ci_vlim = std::dynamic_pointer_cast<VelocityLimits>(constr);

    if(!_ci_vlim)
    {
        throw std::runtime_error("Provided constraint description "
                                 "does not have expected type 'VelocityLimits'");
    }

}

ConstraintPtr OpenSotVelocityLimitsAdapter::constructConstraint()
{
    Eigen::VectorXd q;
    _model->getJointPosition(q);

    _opensot_vlim = std::make_shared<VelocityLimitsSoT>(const_cast<ModelInterface&>(*_model),
                                                          _vars.getVariable("qddot"),
                                                          _ci_vlim->getQdotMax(),
                                                          _ctx->params()->getControlPeriod());

    return _opensot_vlim;
}

bool OpenSotVelocityLimitsAdapter::initialize(const OpenSoT::OptvarHelper& vars)
{
    return OpenSotConstraintAdapter::initialize(vars);
}

void OpenSotVelocityLimitsAdapter::update(double time, double period)
{
    OpenSotConstraintAdapter::update(time, period);
}


OpenSoT::OptvarHelper::VariableVector OpenSotVelocityLimitsAdapter::getRequiredVariables() const
{
    return {{"qddot", _model->getJointNum()}};
}

CARTESIO_REGISTER_OPENSOT_CONSTR_PLUGIN(OpenSotVelocityLimitsAdapter, VelocityLimits)

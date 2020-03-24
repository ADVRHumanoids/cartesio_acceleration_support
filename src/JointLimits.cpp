#include "JointLimits.h"

#include <boost/make_shared.hpp>
#include <fmt/format.h>

using namespace XBot::Cartesian::acceleration;

JointLimitsAccImpl::JointLimitsAccImpl(YAML::Node yaml,
                                       Context::ConstPtr context):
    JointLimitsImpl(yaml, context)
{
    _qddot_max.setConstant(_model->getJointNum(), 1000.0);

    if(yaml["limits_acc"] && yaml["limits_acc"].IsMap())
    {
        for(auto lim : yaml["limits_acc"])
        {
            auto jname = lim.first.as<std::string>();
            auto lim_value = lim.second.as<double>();

            if(!_model->hasJoint(jname))
            {
                throw std::invalid_argument(fmt::format("Invalid joint '{}' in joint acc limits", jname));
            }

            if(lim_value < 0)
            {
                throw std::invalid_argument(fmt::format("Invalid acc limit for joint '{}': "
                                                        "qddot_max ({}) < 0",
                                                        jname, lim_value));
            }

            int idx = _model->getDofIndex(jname);

            _qddot_max[idx] = lim_value;

        }
    }

    if(yaml["limits_acc"] && yaml["limits_acc"].IsScalar())
    {
        _qddot_max.setConstant(_model->getJointNum(), yaml["limits_acc"].as<double>());
    }
}

const Eigen::VectorXd& JointLimitsAccImpl::getQddotMax() const
{
    return _qddot_max;
}



OpenSotJointLimitsAdapter::OpenSotJointLimitsAdapter(ConstraintDescription::Ptr constr,
                                                     Context::ConstPtr context):
    OpenSotConstraintAdapter (constr, context)
{
    _ci_jlim = std::dynamic_pointer_cast<JointLimitsAccImpl>(constr);

    if(!_ci_jlim)
    {
        throw std::runtime_error("Provided constraint description "
                                 "does not have expected type 'JointLimitsAccImpl'");
    }

}

ConstraintPtr OpenSotJointLimitsAdapter::constructConstraint()
{
    Eigen::VectorXd q;
    _model->getJointPosition(q);

    return boost::make_shared<JointLimitsSoT>(const_cast<ModelInterface&>(*_model),
                                              _vars.getVariable("qddot"),
                                              _ci_jlim->getQmax(),
                                              _ci_jlim->getQmin(),
                                              _ci_jlim->getQddotMax(),
                                              _ctx->params()->getControlPeriod());
}

bool OpenSotJointLimitsAdapter::initialize(const OpenSoT::OptvarHelper& vars)
{
    return OpenSotConstraintAdapter::initialize(vars);
}

void OpenSotJointLimitsAdapter::update(double time, double period)
{
    OpenSotConstraintAdapter::update(time, period);
}


OpenSoT::OptvarHelper::VariableVector OpenSotJointLimitsAdapter::getRequiredVariables() const
{
    return {{"qddot", _model->getJointNum()}};
}

CARTESIO_REGISTER_TASK_PLUGIN(JointLimitsAccImpl, JointLimits)
CARTESIO_REGISTER_OPENSOT_CONSTR_PLUGIN(OpenSotJointLimitsAdapter, JointLimits)

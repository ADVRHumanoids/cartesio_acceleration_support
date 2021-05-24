#include "TorqueLimits.h"

using namespace XBot::Cartesian::acceleration;

TorqueLimitsImpl::TorqueLimitsImpl(YAML::Node node, Context::ConstPtr context):
    TaskDescriptionImpl (node, context, "torque_limits", context->model()->getJointNum())
{
    _contact_links = node["contacts"].as<std::vector<std::string>>();

    context->model()->getEffortLimits(_tau_lims);
    for(unsigned int i = 0; i < 6; ++i)
        _tau_lims[i] = 0.;
}

const Eigen::VectorXd& TorqueLimitsImpl::getLimits() const
{
    return _tau_lims;
}

void TorqueLimitsImpl::setLimits(Eigen::VectorXd& tau_lims)
{
    _tau_lims = tau_lims;
}

const std::vector<std::string>& TorqueLimitsImpl::getLinksInContact() const
{
    return _contact_links;
}

OpenSotTorqueLimitsAdapter::OpenSotTorqueLimitsAdapter(ConstraintDescription::Ptr constr,
                                                       Context::ConstPtr context):
    OpenSotConstraintAdapter(constr, context)
{
    _ci_taulim = std::dynamic_pointer_cast<TorqueLimits>(constr);
    if(!_ci_taulim) throw std::runtime_error("Provided constraint description "
                                            "does not have expected type 'TorqueLimits'");
}

CARTESIO_REGISTER_TASK_PLUGIN(TorqueLimitsImpl, TorqueLimits)
CARTESIO_REGISTER_OPENSOT_CONSTR_PLUGIN(OpenSotTorqueLimitsAdapter, TorqueLimits)

ConstraintPtr OpenSotTorqueLimitsAdapter::constructConstraint()
{
    std::vector<OpenSoT::AffineHelper> cl_vars;

    for(auto cl : _ci_taulim->getLinksInContact())
    {
        cl_vars.push_back(_vars.getVariable("force_" + cl));
    }

    OpenSoT::AffineHelper qddot;
    qddot.setZero(_vars.getAllVariables().front().getInputSize(),
                  _model->getJointNum());

    qddot = _vars.getVariable("qddot");


    _opensot_taulim = std::make_shared<TaulimSoT>(*_model,
                                                    qddot,
                                                    cl_vars,
                                                    _ci_taulim->getLinksInContact(),
                                                    _ci_taulim->getLimits());

    return _opensot_taulim;
}

OpenSoT::OptvarHelper::VariableVector OpenSotTorqueLimitsAdapter::getRequiredVariables() const
{
    OpenSoT::OptvarHelper::VariableVector vars;

    for(auto cl : _ci_taulim->getLinksInContact())
    {
        vars.emplace_back("force_" + cl, 6); ///TODO: Here we assume planar contacts!
    }

    vars.emplace_back("qddot", _model->getJointNum());

    return vars;
}

void OpenSotTorqueLimitsAdapter::update(double time, double period)
{
    OpenSotConstraintAdapter::update(time, period);

    _opensot_taulim->setTorqueLimits(_ci_taulim->getLimits());
}

#include "TorqueLimits.hpp"

using namespace XBot::Cartesian::acceleration;

TorqueLimitsImpl::TorqueLimitsImpl(YAML::Node node, Context::ConstPtr context):
    TaskDescriptionImpl (node, context, "torque_limits", context->model()->getJointNum())
{
    _contact_links = node["contacts"].as<std::vector<std::string>>();

    context->model()->getEffortLimits(_tau_lims);
    for(unsigned int i = 0; i < 6; ++i)
        _tau_lims[i] = 0.;

    _contact_model = OpenSoT::utils::InverseDynamics::CONTACT_MODEL::SURFACE_CONTACT;
    if(auto contact_model = node["contact_model"])
    {
        auto cm = contact_model.as<std::string>();
        if(cm == "point")
            _contact_model = OpenSoT::utils::InverseDynamics::CONTACT_MODEL::POINT_CONTACT;
        else if(cm == "surface")
            _contact_model = OpenSoT::utils::InverseDynamics::CONTACT_MODEL::SURFACE_CONTACT;
        else
            throw std::invalid_argument("supported contact models are ´point´ and ´surface´");
    }
}

OpenSoT::utils::InverseDynamics::CONTACT_MODEL TorqueLimitsImpl::getContactModel()
{
    return _contact_model;
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
    _ci_taulim = std::dynamic_pointer_cast<TorqueLimitsImpl>(constr);
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
                  _model->getNv());

    qddot = _vars.getVariable("qddot");


    _opensot_taulim = SotUtils::make_shared<TaulimSoT>(*_model,
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
        if(_ci_taulim->getContactModel() == OpenSoT::utils::InverseDynamics::CONTACT_MODEL::SURFACE_CONTACT)
            vars.emplace_back("force_" + cl, 6);
        else
            vars.emplace_back("force_" + cl, 3);
    }

    vars.emplace_back("qddot", _model->getNv());

    return vars;
}

void OpenSotTorqueLimitsAdapter::update(double time, double period)
{
    OpenSotConstraintAdapter::update(time, period);

    _opensot_taulim->setTorqueLimits(_ci_taulim->getLimits());
}

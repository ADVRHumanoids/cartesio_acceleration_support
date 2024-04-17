#include "DynamicFeasibility.hpp"
#include <boost/make_shared.hpp>

using namespace XBot::Cartesian::acceleration;

DynamicFeasibilityImpl::DynamicFeasibilityImpl(YAML::Node task_node,
                                               Context::ConstPtr context):
    TaskDescriptionImpl(task_node, context, "DynamicFeasibility", 6),
    _dynamics(true)
{
    _contact_links = task_node["contacts"].as<std::vector<std::string>>();

    if(auto dyn = task_node["dynamics"])
    {
        _dynamics = dyn.as<bool>();
    }

    _contact_model = OpenSoT::utils::InverseDynamics::CONTACT_MODEL::SURFACE_CONTACT;
    if(auto contact_model = task_node["contact_model"])
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

std::vector<std::string> DynamicFeasibilityImpl::getContactLinks() const
{
    return _contact_links;
}

bool DynamicFeasibilityImpl::dynamicsEnabled() const
{
    return _dynamics;
}

OpenSoT::utils::InverseDynamics::CONTACT_MODEL DynamicFeasibilityImpl::getContactModel()
{
    return _contact_model;
}

OpenSotDynFeasAdapter::OpenSotDynFeasAdapter(TaskDescription::Ptr task,
                                             Context::ConstPtr context):
    OpenSotTaskAdapter (task, context)
{
    _ci_dynfeas = std::dynamic_pointer_cast<DynamicFeasibilityImpl>(task);
    if(!_ci_dynfeas) throw std::runtime_error("Provided task description "
                                              "does not have expected type 'DynamicFeasibilityImpl'");
}

OpenSoT::OptvarHelper::VariableVector OpenSotDynFeasAdapter::getRequiredVariables() const
{
    OpenSoT::OptvarHelper::VariableVector vars;

    for(auto cl : _ci_dynfeas->getContactLinks())
    {
        if(_ci_dynfeas->getContactModel() == OpenSoT::utils::InverseDynamics::CONTACT_MODEL::SURFACE_CONTACT)
            vars.emplace_back("force_" + cl, 6);
        else
            vars.emplace_back("force_" + cl, 3);
    }

    if(_ci_dynfeas->dynamicsEnabled())
    {
        vars.emplace_back("qddot", _model->getNv());
    }

    return vars;
}

TaskPtr OpenSotDynFeasAdapter::constructTask()
{
    std::vector<OpenSoT::AffineHelper> cl_vars;

    for(auto cl : _ci_dynfeas->getContactLinks())
    {
        cl_vars.push_back(_vars.getVariable("force_" + cl));
    }

    OpenSoT::AffineHelper qddot;
    qddot.setZero(_vars.getAllVariables().front().getInputSize(),
                  _model->getNv());

    if(_ci_dynfeas->dynamicsEnabled())
    {
        qddot = _vars.getVariable("qddot");
    }

    _opensot_dynfeas = SotUtils::make_shared<DynamicFeasibilitySoT>("dynamic_feas",
                                                                 *_model,
                                                                 qddot,
                                                                 cl_vars,
                                                                 _ci_dynfeas->getContactLinks());

    return _opensot_dynfeas;
}

CARTESIO_REGISTER_TASK_PLUGIN(DynamicFeasibilityImpl, DynamicFeasibility)
CARTESIO_REGISTER_OPENSOT_TASK_PLUGIN(OpenSotDynFeasAdapter, DynamicFeasibility)

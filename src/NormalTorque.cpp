#include "NormalTorque.h"

using namespace XBot::Cartesian::acceleration;

namespace {

std::string get_name(YAML::Node node)
{
    auto link = node["link"].as<std::string>();
    return "normal_torque_" + link;
}


}

NormalTorqueImpl::NormalTorqueImpl(YAML::Node node,
                                   Context::ConstPtr context):
    TaskDescriptionImpl(node, context, get_name(node), 8)
{
    _link = node["link"].as<std::string>();

    if(auto xlims = node["x_limits"])
    {
        auto tmp = xlims.as<double>();
        _x_lim[0] = -tmp;
        _x_lim[1] = tmp;
    }
    else
        throw std::invalid_argument("Mandatory field 'x_limits' is missing!");

    if(auto ylims = node["y_limits"])
    {
        auto tmp = ylims.as<double>();
        _y_lim[0] = -tmp;
        _y_lim[1] = tmp;
    }
    else
        throw std::invalid_argument("Mandatory field 'y_limits' is missing!");

    if(auto ylims = node["friction_coeff"])
    {
        auto tmp = ylims.as<double>();
        _mu = tmp;
    }
    else
        throw std::invalid_argument("Mandatory field 'friction_coeff' is missing!");
}

const std::string& NormalTorqueImpl::getLinkName() const
{
    return _link;
}

const Eigen::Vector2d& NormalTorqueImpl::getXLims() const
{
    return _x_lim;
}

const Eigen::Vector2d& NormalTorqueImpl::getYLims() const
{
    return _y_lim;
}

double NormalTorqueImpl::getFrictionCoeff()
{
    return _mu;
}


OpenSotNormalTorqueAdapter::OpenSotNormalTorqueAdapter(ConstraintDescription::Ptr constr, Context::ConstPtr context):
    OpenSotConstraintAdapter(constr, context)
{
    _ci_nt = std::dynamic_pointer_cast<NormalTorque>(constr);
    if(!constr) throw std::runtime_error("Provided task description "
                                         "does not have expected type 'NormalTorque'");


    _var_name = "force_" + _ci_nt->getLinkName();
}

ConstraintPtr OpenSotNormalTorqueAdapter::constructConstraint()
{
    _opensot_nt = boost::make_shared<NormalTorqueSoT>(
                _ci_nt->getLinkName(),
                _vars.getVariable(_var_name),
                const_cast<ModelInterface&>(*_model),
                _ci_nt->getXLims()[1], _ci_nt->getYLims()[1],
                _ci_nt->getFrictionCoeff());

    return _opensot_nt;
}

OpenSoT::OptvarHelper::VariableVector OpenSotNormalTorqueAdapter::getRequiredVariables() const
{
    return {{_var_name, 6}};
}

void OpenSotNormalTorqueAdapter::update(double time, double period)
{
    OpenSotConstraintAdapter::update(time, period);
}

CARTESIO_REGISTER_TASK_PLUGIN(NormalTorqueImpl, NormalTorque)
CARTESIO_REGISTER_OPENSOT_CONSTR_PLUGIN(OpenSotNormalTorqueAdapter, NormalTorque)


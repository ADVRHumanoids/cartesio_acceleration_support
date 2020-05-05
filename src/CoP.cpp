#include "CoP.h"

using namespace XBot::Cartesian::acceleration;

namespace  {

std::string get_name(YAML::Node node)
{
    auto link = node["link"].as<std::string>();
    return "cop_" + link;
}

}


CoPImpl::CoPImpl(YAML::Node node,
                                   Context::ConstPtr context):
    TaskDescriptionImpl(node, context, get_name(node), 4)
{
    _link = node["link"].as<std::string>();

    if(auto xlims = node["x_limits"])
    {
        auto tmp = xlims.as<std::vector<double>>();
        if(tmp.size() != 2)
            throw std::invalid_argument("Field 'x_limits' size must be 2");
        _x_lim[0] = tmp[0];
        _x_lim[1] = tmp[1];
    }
    else
        throw std::invalid_argument("Mandatory field 'x_limits' is missing!");

    if(auto ylims = node["y_limits"])
    {
        auto tmp = ylims.as<std::vector<double>>();
        if(tmp.size() != 2)
            throw std::invalid_argument("Field 'y_limits' size must be 2");
        _y_lim[0] = tmp[0];
        _y_lim[1] = tmp[1];
    }
    else
        throw std::invalid_argument("Mandatory field 'y_limits' is missing!");
}

const std::string& CoPImpl::getLinkName() const
{
    return _link;
}

const Eigen::Vector2d& CoPImpl::getXLims() const
{
    return _x_lim;
}

const Eigen::Vector2d& CoPImpl::getYLims() const
{
    return _y_lim;
}

OpenSotCoPAdapter::OpenSotCoPAdapter(ConstraintDescription::Ptr constr, Context::ConstPtr context):
    OpenSotConstraintAdapter(constr, context)
{
    _ci_cop = std::dynamic_pointer_cast<CoP>(constr);
    if(!constr) throw std::runtime_error("Provided task description "
                                         "does not have expected type 'CoP'");


    _var_name = "force_" + _ci_cop->getLinkName();
}

ConstraintPtr OpenSotCoPAdapter::constructConstraint()
{
    _opensot_cop = boost::make_shared<CoPSoT>(_ci_cop->getLinkName(),
                                            _vars.getVariable(_var_name),
                                            const_cast<ModelInterface&>(*_model),
                                            _ci_cop->getXLims(), _ci_cop->getYLims());

    return _opensot_cop;
}

OpenSoT::OptvarHelper::VariableVector OpenSotCoPAdapter::getRequiredVariables() const
{
    return {{_var_name, 4}};
}

void OpenSotCoPAdapter::update(double time, double period)
{
    OpenSotConstraintAdapter::update(time, period);
}

CARTESIO_REGISTER_TASK_PLUGIN(CoPImpl, CoP)
CARTESIO_REGISTER_OPENSOT_CONSTR_PLUGIN(OpenSotCoPAdapter, CoP)


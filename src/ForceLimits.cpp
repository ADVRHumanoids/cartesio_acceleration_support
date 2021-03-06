#include "ForceLimits.h"
#include <std_srvs/SetBool.h>

#include <fmt/format.h>

using namespace XBot::Cartesian::acceleration;

namespace  {

std::string get_name(YAML::Node node)
{
    auto link = node["link"].as<std::string>();
    return "force_lims_" + link;
}

}


ForceLimitsImpl::ForceLimitsImpl(YAML::Node node,
                                 Context::ConstPtr context):
    TaskDescriptionImpl (node, context, get_name(node), 6),
    _local(false),
    _zeroed(false)
{
    if(auto n = node["min"])
    {
        auto vec = n.as<std::vector<double>>();

        if(vec.size() != 6)
        {
            throw std::invalid_argument(
                        fmt::format("Size mismatch in '{}': 'min' field must have dimension 6",
                                    getName())
                        );
        }

        _fmin = Eigen::Vector6d::Map(vec.data());
    }
    else
    {
        throw std::runtime_error(
                    fmt::format("Missing required field 'min' in '{}'",
                                getName())
                    );
    }

    if(auto n = node["max"])
    {
        auto vec = n.as<std::vector<double>>();

        if(vec.size() != 6)
        {
            throw std::invalid_argument(
                        fmt::format("Size mismatch for '{}': 'max' field must have dimension 6",
                                    getName())
                        );
        }

        _fmax = Eigen::Vector6d::Map(vec.data());
    }
    else
    {
        throw std::runtime_error(
                    fmt::format("Missing required field 'min' in '{}'",
                                getName())
                    );
    }

    if(auto n = node["link"])
    {
        _link = n.as<std::string>();
    }
    else
    {
        throw std::runtime_error(
                    fmt::format("Missing required field 'link' in '{}'",
                                getName())
                    );
    }

    setLimits(_fmin, _fmax);

    if(auto n = node["local"])
    {
        _local = n.as<bool>();
    }

}

const std::string& ForceLimitsImpl::getLinkName() const
{
    return _link;
}

bool ForceLimitsImpl::isLocal() const
{
    return _local;
}

void ForceLimitsImpl::getLimits(Eigen::Vector6d& fmin, Eigen::Vector6d& fmax) const
{
    if(_zeroed)
    {
        fmin.setZero();
        fmax.setZero();
        return;
    }

    fmin = _fmin;
    fmax = _fmax;
}

void ForceLimitsImpl::setLimits(const Eigen::Vector6d& fmin, const Eigen::Vector6d& fmax)
{
    if((fmax-fmin).minCoeff() < 0)
    {
        throw std::invalid_argument("fmin must be smaller-equal than fmax");
    }

    _fmin = fmin;
    _fmax = fmax;
}

void XBot::Cartesian::acceleration::ForceLimitsImpl::setZero()
{
    _zeroed = true;
}

void XBot::Cartesian::acceleration::ForceLimitsImpl::restore()
{
    _zeroed = false;
}

OpenSotForceLimitsAdapter::OpenSotForceLimitsAdapter(ConstraintDescription::Ptr constr,
                                                     Context::ConstPtr context):
    OpenSotConstraintAdapter(constr, context)
{
    _ci_flim = std::dynamic_pointer_cast<ForceLimits>(constr);
    if(!_ci_flim) throw std::runtime_error("Provided task description "
                                            "does not have expected type 'ForceLimits'");


    _var_name = "force_" + _ci_flim->getLinkName();
}

ConstraintPtr OpenSotForceLimitsAdapter::constructConstraint()
{
    Eigen::Vector6d fmin, fmax;
    _ci_flim->getLimits(fmin, fmax);

    _opensot_flim = boost::make_shared<FlimSoT>(
                _ci_flim->getLinkName(),
                fmin, fmax,
                _vars.getVariable(_var_name));

    return _opensot_flim;
}

OpenSoT::OptvarHelper::VariableVector OpenSotForceLimitsAdapter::getRequiredVariables() const
{
    return {{_var_name, 6}};
}

void OpenSotForceLimitsAdapter::update(double time, double period)
{
    OpenSotConstraintAdapter::update(time, period);

    Eigen::Vector6d fmin, fmax;
    _ci_flim->getLimits(fmin, fmax);

    _opensot_flim->setWrenchLimits(fmin, fmax);
}


ForceLimitsRos::ForceLimitsRos(TaskDescription::Ptr task,
                               RosContext::Ptr context):
    TaskRos(task, context)
{
    _ci_force = std::dynamic_pointer_cast<ForceLimits>(task);
    if(!_ci_force) throw std::runtime_error("Provided task description "
                                            "does not have expected type 'ForceLimits'");

    using Req = std_srvs::SetBoolRequest;
    using Res = std_srvs::SetBoolResponse;

    auto toggle_cb = [this](Req& req, Res& res)
    {
        if(req.data)
        {
            res.message = fmt::format("Successfully restored previous limits for '{}'",
                                      _ci_force->getName());
            _ci_force->restore();
        }
        else
        {
            res.message = fmt::format("Successfully zeroed limits for '{}'",
                                      _ci_force->getName());
            _ci_force->setZero();
        }

        res.success = true;
        return true;
    };

    _toggle_srv = _ctx->nh().advertiseService<Req,Res>(task->getName() + "/toggle_contact",
                                                       toggle_cb);

    /* Register type name */
    registerType("ForceLimits");
}

CARTESIO_REGISTER_TASK_PLUGIN(ForceLimitsImpl, ForceLimits)
CARTESIO_REGISTER_OPENSOT_CONSTR_PLUGIN(OpenSotForceLimitsAdapter, ForceLimits)
CARTESIO_REGISTER_ROS_API_PLUGIN(ForceLimitsRos, ForceLimits)



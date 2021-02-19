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

    _ci_force->getLimits(_flim_min, _flim_max);
    auto on_flim_min_recv = [this](geometry_msgs::WrenchConstPtr msg)
    {
        tf::wrenchMsgToEigen(*msg, _flim_min);
        _ci_force->setLimits(_flim_min, _flim_max);
    };
    auto on_flim_max_recv = [this](geometry_msgs::WrenchConstPtr msg)
    {
        tf::wrenchMsgToEigen(*msg, _flim_max);
        _ci_force->setLimits(_flim_min, _flim_max);
    };

    _flim_min_sub = _ctx->nh().subscribe<geometry_msgs::Wrench>(task->getName() + "/limits_min", 5, on_flim_min_recv);
    _flim_max_sub = _ctx->nh().subscribe<geometry_msgs::Wrench>(task->getName() + "/limits_max", 5, on_flim_max_recv);

    /* Register type name */
    registerType("ForceLimits");
}

ForceLimitsRosClient::ForceLimitsRosClient(std::string name, ros::NodeHandle nh):
TaskRos(name, nh)
{
    _link_name = name.substr(11);

    auto on_f_lim_min_recv = [this](geometry_msgs::WrenchConstPtr msg)
    {
        tf::wrenchMsgToEigen(*msg, _flim_min_value);
    };
    auto on_f_lim_max_recv = [this](geometry_msgs::WrenchConstPtr msg)
    {
        tf::wrenchMsgToEigen(*msg, _flim_max_value);
    };

    _flim_min_sub = _nh.subscribe<geometry_msgs::Wrench>(name + "/value_min", 10,
                                                              on_f_lim_min_recv);
    _flim_max_sub = _nh.subscribe<geometry_msgs::Wrench>(name + "/value_max", 10,
                                                              on_f_lim_max_recv);

    _flim_min_pub = _nh.advertise<geometry_msgs::Wrench>(name + "/limits_min", 10, true);
    _flim_max_pub = _nh.advertise<geometry_msgs::Wrench>(name + "/limits_max", 10, true);
}

const std::string& ForceLimitsRosClient::getLinkName() const
{
    return _link_name;
}

void ForceLimitsRosClient::setLimits(const Eigen::Vector6d &fmin, const Eigen::Vector6d &fmax)
{
    geometry_msgs::Wrench msg_min, msg_max;

    tf::wrenchEigenToMsg(fmin, msg_min);
    tf::wrenchEigenToMsg(fmax, msg_max);

    _flim_min_pub.publish(msg_min);
    _flim_max_pub.publish(msg_max);
}

void ForceLimitsRosClient::getLimits(Eigen::Vector6d &fmin, Eigen::Vector6d &fmax) const
{
    fmin = _flim_min_value;
    fmax = _flim_max_value;
}

void ForceLimitsRosClient::setZero()
{
    _flim_min_value.setZero();
    _flim_max_value.setZero();
}

void ForceLimitsRosClient::restore()
{
    throw std::runtime_error("Unsupported " + std::string() + __func__);
}

bool ForceLimitsRosClient::isLocal() const
{
    throw std::runtime_error("Unsupported " + std::string() + __func__);
}

CARTESIO_REGISTER_TASK_PLUGIN(ForceLimitsImpl, ForceLimits)
CARTESIO_REGISTER_OPENSOT_CONSTR_PLUGIN(OpenSotForceLimitsAdapter, ForceLimits)
CARTESIO_REGISTER_ROS_CLIENT_API_PLUGIN(ForceLimitsRosClient, ForceLimits)
CARTESIO_REGISTER_ROS_API_PLUGIN(ForceLimitsRos, ForceLimits)



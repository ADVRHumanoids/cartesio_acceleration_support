#include "Force.h"

#include <geometry_msgs/WrenchStamped.h>
#include <visualization_msgs/Marker.h>
#include <eigen_conversions/eigen_msg.h>

using namespace XBot::Cartesian::acceleration;

namespace  {

std::string get_name(YAML::Node node)
{
    auto link = node["link"].as<std::string>();
    return "force_" + link;
}

}

ForceTaskImpl::ForceTaskImpl(YAML::Node node,
                             Context::ConstPtr context):
    TaskDescriptionImpl (node, context, get_name(node), 6)
{
    _link = node["link"].as<std::string>();
    _fref.setZero();
    _fvalue = _fref;
    _T.setIdentity();
}

const std::string& ForceTaskImpl::getLinkName() const
{
    return _link;
}

const Eigen::Vector6d& ForceTaskImpl::getForceReference() const
{
    return _fref;
}

Eigen::Affine3d ForceTaskImpl::getForceFrame() const
{
    return _T;
}

void ForceTaskImpl::setForceReference(const Eigen::Vector6d& f)
{
    _fref = f;
}

const Eigen::Vector6d& ForceTaskImpl::getForceValue() const
{
    return _fvalue;
}

void ForceTaskImpl::setForceValue(const Eigen::Vector6d& f)
{
    _fvalue = f;
}

void ForceTaskImpl::setForceFrame(const Eigen::Affine3d& T)
{
    _T = T;
}

ForceTaskRos::ForceTaskRos(TaskDescription::Ptr task,
                           RosContext::Ptr context):
    TaskRos(task, context)
{
    _ci_force = std::dynamic_pointer_cast<ForceTask>(task);
    if(!_ci_force) throw std::runtime_error("Provided task description "
                                            "does not have expected type 'ForceTask'");

    _f_pub = _ctx->nh().advertise<geometry_msgs::WrenchStamped>(task->getName() + "/value",
                                                                1);
    _f_fpub_visual = _ctx->nh().advertise<visualization_msgs::Marker>(task->getName() + "/force/value", 1);
    _f_tpub_visual = _ctx->nh().advertise<visualization_msgs::Marker>(task->getName() + "/torque/value", 1);

    auto on_fref_recv = [this](geometry_msgs::WrenchStampedConstPtr msg)
    {
        Eigen::Vector6d fref;
        tf::wrenchMsgToEigen(msg->wrench, fref);
        _ci_force->setForceReference(fref);
    };

    _fref_sub = _ctx->nh().subscribe<geometry_msgs::WrenchStamped>(
                task->getName() + "/reference", 5,
                on_fref_recv);

    /* Register type name */
    registerType("Force");
}

void ForceTaskRos::run(ros::Time time)
{
    TaskRos::run(time);

    geometry_msgs::WrenchStamped msg;
    auto f = _ci_force->getForceValue();
    f.head<3>() = _ci_force->getForceFrame().linear().transpose() * f.head<3>();
    f.tail<3>() = _ci_force->getForceFrame().linear().transpose() * f.tail<3>();
    tf::wrenchEigenToMsg(f, msg.wrench);
    msg.header.stamp = time;
    msg.header.frame_id = _ctx->tf_prefix_slash() + _ci_force->getLinkName();
    _f_pub.publish(msg);

    visualization_msgs::Marker msg_force, msg_torque;
    f = _ci_force->getForceValue();
    f.head<3>() = _ci_force->getForceFrame().linear().transpose() * f.head<3>();
    f.tail<3>() = _ci_force->getForceFrame().linear().transpose() * f.tail<3>();
    f = f * 0.005;
    msg_force.header.stamp = time;
    msg_force.header.frame_id = _ctx->tf_prefix_slash() + _ci_force->getLinkName();
    msg_force.type = visualization_msgs::Marker::ARROW;
    msg_force.action = visualization_msgs::Marker::ADD;
    msg_force.points.resize(2);
    msg_force.points[0].x = 0;  msg_force.points[0].y = 0;  msg_force.points[0].z = 0;
    msg_force.points[1].x = f(0);   msg_force.points[1].y = f(1);   msg_force.points[1].z = f(2);
    msg_force.color.r = 1;   msg_force.color.b = 0;   msg_force.color.g = 0;   msg_force.color.a = 1;
    msg_force.scale.x = 0.05;    msg_force.scale.y = 0.075;    msg_force.scale.z = 0.2;
    _f_fpub_visual.publish(msg_force);

    msg_torque.header.stamp = time;
    msg_torque.header.frame_id = _ctx->tf_prefix_slash() + _ci_force->getLinkName();
    msg_torque.type = visualization_msgs::Marker::ARROW;
    msg_torque.action = visualization_msgs::Marker::ADD;
    msg_torque.points.resize(2);
    msg_torque.points[0].x = 0;   msg_torque.points[0].y = 0;   msg_torque.points[0].z = 0;
    msg_torque.points[1].x = f(3);   msg_torque.points[1].y = f(4);   msg_torque.points[1].z = f(5);
    msg_torque.color.r = 0;   msg_torque.color.b = 1;   msg_torque.color.g = 0;   msg_torque.color.a = 1;
    msg_torque.scale.x = 0.05;    msg_torque.scale.y = 0.075;    msg_torque.scale.z = 0.2;
    _f_tpub_visual.publish(msg_torque);
}


OpenSotForceAdapter::OpenSotForceAdapter(TaskDescription::Ptr ci_task,
                                         Context::ConstPtr context):
    OpenSotTaskAdapter(ci_task, context)
{
    _ci_force = std::dynamic_pointer_cast<ForceTask>(ci_task);
    if(!_ci_force) throw std::runtime_error("Provided task description "
                                            "does not have expected type 'ForceTask'");


    _var_name = "force_" + _ci_force->getLinkName();
}

OpenSoT::OptvarHelper::VariableVector OpenSotForceAdapter::getRequiredVariables() const
{
    return {{_var_name, 6}};
}

TaskPtr OpenSotForceAdapter::constructTask()
{
    _var = _vars.getVariable(_var_name);

    _opensot_wrench = SotUtils::make_shared<WrenchSoT>(_var_name + "_task",
                                                    _ci_force->getLinkName(),
                                                    "arg_unused",
                                                    _var);

    return _opensot_wrench;
}

void OpenSotForceAdapter::update(double time, double period)
{
    OpenSotTaskAdapter::update(time, period);

    _opensot_wrench->setReference(_ci_force->getForceReference());
}

void OpenSotForceAdapter::processSolution(const Eigen::VectorXd& solution)
{
    OpenSotTaskAdapter::processSolution(solution);

    Eigen::Vector6d f;
    _var.getValue(solution, f);
    _ci_force->setForceValue(f);

    Eigen::Affine3d T;
    _model->getPose(_ci_force->getLinkName(), T);
    _ci_force->setForceFrame(T);
}



ForceTaskRosClient::ForceTaskRosClient(std::string name, ros::NodeHandle nh):
    TaskRos(name, nh)
{
    _link_name = name.substr(6);

    auto on_fvalue_recv = [this](geometry_msgs::WrenchStampedConstPtr msg)
    {
        tf::wrenchMsgToEigen(msg->wrench, _fvalue);
    };

    _fvalue_sub = _nh.subscribe<geometry_msgs::WrenchStamped>(name + "/value", 1,
                                                              on_fvalue_recv);

    _fref_pub = _nh.advertise<geometry_msgs::WrenchStamped>(name + "/reference", 1);
}

const string& ForceTaskRosClient::getLinkName() const
{
    return _link_name;
}

const Eigen::Vector6d& ForceTaskRosClient::getForceReference() const
{
    throw std::runtime_error("Unsupported " + std::string() + __func__);
}

const Eigen::Vector6d& ForceTaskRosClient::getForceValue() const
{
    return _fvalue;
}

Eigen::Affine3d ForceTaskRosClient::getForceFrame() const
{
    throw std::runtime_error("Unsupported " + std::string() + __func__);
}

void ForceTaskRosClient::setForceReference(const Eigen::Vector6d& f)
{
    geometry_msgs::WrenchStamped msg;
    tf::wrenchEigenToMsg(f, msg.wrench);
    msg.header.frame_id = "ci/" + _link_name; // TBD dynamic tf prefix
    msg.header.stamp = ros::Time::now();

    _fref_pub.publish(msg);

}

void ForceTaskRosClient::setForceValue(const Eigen::Vector6d& f)
{
    throw std::runtime_error("Unsupported " + std::string() + __func__);
}

void ForceTaskRosClient::setForceFrame(const Eigen::Affine3d& T)
{
    throw std::runtime_error("Unsupported " + std::string() + __func__);
}

CARTESIO_REGISTER_TASK_PLUGIN(ForceTaskImpl, Force)
CARTESIO_REGISTER_ROS_API_PLUGIN(ForceTaskRos, Force)
CARTESIO_REGISTER_ROS_CLIENT_API_PLUGIN(ForceTaskRosClient, Force)
CARTESIO_REGISTER_OPENSOT_TASK_PLUGIN(OpenSotForceAdapter, Force)

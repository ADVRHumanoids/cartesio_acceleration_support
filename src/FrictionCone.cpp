#include "FrictionCone.h"

#include <fmt/format.h>

using namespace XBot::Cartesian::acceleration;

namespace  {

std::string get_name(YAML::Node node)
{
    auto link = node["link"].as<std::string>();
    return "friction_cone_" + link;
}

}

FrictionConeImpl::FrictionConeImpl(YAML::Node node,
                                   Context::ConstPtr context):
    TaskDescriptionImpl(node, context, get_name(node), 5)
{
    _link = node["link"].as<std::string>();
    _local = false;
    _R.setIdentity();
    _mu = 1.0;

    if(auto loc = node["local"])
    {
        _local = loc.as<bool>();
    }

    if(auto mu = node["friction_coeff"])
    {
        _mu = mu.as<double>();
    }

    if(auto rot = node["contact_frame"])
    {
        auto quat = rot.as<std::vector<double>>();

        if(quat.size() != 4)
        {
            throw std::invalid_argument("Field 'contact_frame' must be a quaternion");
        }

        _R = Eigen::Quaterniond(Eigen::Vector4d::Map(quat.data())).toRotationMatrix();
    }

}


const std::string& FrictionConeImpl::getLinkName() const
{
    return _link;
}

bool FrictionConeImpl::isLocal() const
{
    return _local;
}

double FrictionConeImpl::getFrictionCoeff() const
{
    return _mu;
}

void FrictionConeImpl::setFrictionCoeff(const double mu)
{
    _mu = mu;
}

Eigen::Matrix3d FrictionConeImpl::getContactFrame() const
{
    return _R;
}

void FrictionConeImpl::setContactRotationMatrix(const Eigen::Matrix3d &R)
{
    _R = R;
}



OpenSotFrictionConeAdapter::OpenSotFrictionConeAdapter(ConstraintDescription::Ptr constr,
                                                       Context::ConstPtr context):
    OpenSotConstraintAdapter(constr, context)
{
    _ci_fc = std::dynamic_pointer_cast<FrictionCone>(constr);
    if(!constr) throw std::runtime_error("Provided task description "
                                         "does not have expected type 'FrictionCone'");


    _var_name = "force_" + _ci_fc->getLinkName();
}

ConstraintPtr OpenSotFrictionConeAdapter::constructConstraint()
{
    FcSoT::friction_cone fc(_ci_fc->getContactFrame(), _ci_fc->getFrictionCoeff());

    _opensot_fc = std::make_shared<FcSoT>(_ci_fc->getLinkName(),
                                            _vars.getVariable(_var_name),
                                            const_cast<ModelInterface&>(*_model),
                                            fc);
    _R.setIdentity();
    _mu = 1.;

    return _opensot_fc;
}

OpenSoT::OptvarHelper::VariableVector OpenSotFrictionConeAdapter::getRequiredVariables() const
{
    return {{_var_name, 6}};
}

void OpenSotFrictionConeAdapter::update(double time, double period)
{
    OpenSotConstraintAdapter::update(time, period);

    if(_ci_fc->isLocal())
    {
        Eigen::Matrix3d w_R_link;
        _model->getOrientation(_ci_fc->getLinkName(), w_R_link);

        Eigen::Matrix3d link_R_c = _ci_fc->getContactFrame();

        _opensot_fc->setContactRotationMatrix(w_R_link * link_R_c);
    }
    else
    {
        if(_R != _ci_fc->getContactFrame())
        {
            _opensot_fc->setContactRotationMatrix(_ci_fc->getContactFrame());
            _R = _ci_fc->getContactFrame();
        }
    }

    if(_mu != _ci_fc->getFrictionCoeff())
    {
        _opensot_fc->setMu(_ci_fc->getFrictionCoeff());
        _mu = _ci_fc->getFrictionCoeff();
    }

}

FrictionConeRos::FrictionConeRos(TaskDescription::Ptr task, RosContext::Ptr context):
    TaskRos(task, context)
{
    _ci_fc = std::dynamic_pointer_cast<FrictionCone>(task);
    if(!_ci_fc) throw std::runtime_error("Provided task description "
                                            "does not have expected type 'ForceLimits'");

    auto on_fc_rcv = [this](std_msgs::Float64::ConstPtr msg)
    {
        double fc = msg->data;
        _ci_fc->setFrictionCoeff(fc);
    };

    auto on_rot_rcv = [this](geometry_msgs::Quaternion::ConstPtr msg)
    {
        Eigen::Quaternion<double> quat;
        tf::quaternionMsgToEigen(*msg, quat);
        Eigen::Matrix3d R;
        R = quat.toRotationMatrix();

        _ci_fc->setContactRotationMatrix(R);
    };

    _fc_pub = _ctx->nh().advertise<std_msgs::Float64>(task->getName() + "/friction_coefficient_value", 10);
    _fc_sub = _ctx->nh().subscribe<std_msgs::Float64>(task->getName() + "/friction_coefficient", 10, on_fc_rcv);
    _rot_pub = _ctx->nh().advertise<geometry_msgs::Quaternion>(task->getName() + "/rotation_value", 10);
    _rot_sub = _ctx->nh().subscribe<geometry_msgs::Quaternion>(task->getName() + "/rotation", 10, on_rot_rcv);

    registerType("FrictionCone");
}

void FrictionConeRos::run(ros::Time time)
{
    TaskRos::run(time);

    geometry_msgs::Quaternion msg_rot;
    std_msgs::Float64 msg_fc;

    Eigen::Matrix3d R;
    double fc;

    fc = _ci_fc->getFrictionCoeff();
    msg_fc.data = fc;

    R = _ci_fc->getContactFrame();
    tf::quaternionEigenToMsg(Eigen::Quaternion<double>(R), msg_rot);

    _fc_pub.publish(msg_fc);
    _rot_pub.publish(msg_rot);
}

FrictionConeRosClient::FrictionConeRosClient(std::string name, ros::NodeHandle nh):
    TaskRos(name, nh)
{
    _link_name = name.substr(15);

    auto on_rotation_mtrx_rcv = [this](geometry_msgs::Quaternion::ConstPtr msg)
    {
        Eigen::Quaternion<double> quat;
        tf::quaternionMsgToEigen(*msg, quat);
        _R = quat.toRotationMatrix();
    };

    auto on_friction_coeff_rcv = [this](std_msgs::Float64::ConstPtr msg)
    {
        _fc = msg->data;
    };

    _rot_sub = _nh.subscribe<geometry_msgs::Quaternion>(name + "/rotation_value", 10, on_rotation_mtrx_rcv);
    _fc_sub = _nh.subscribe<std_msgs::Float64>(name + "/friction_coefficient_value", 10, on_friction_coeff_rcv);

    _rot_pub = _nh.advertise<geometry_msgs::Quaternion>(name + "/rotation", 10, true);
    _fc_pub = _nh.advertise<std_msgs::Float64>(name + "friction_coefficient", 10, true);
}

const std::string& FrictionConeRosClient::getLinkName() const
{
    return _link_name;
}

bool FrictionConeRosClient::isLocal() const
{
    throw std::runtime_error("Unsupported " + std::string() + __func__);
}

double FrictionConeRosClient::getFrictionCoeff() const
{
    return _fc;
}

Eigen::Matrix3d FrictionConeRosClient::getContactFrame() const
{
    return _R;
}

void FrictionConeRosClient::setFrictionCoeff(const double mu)
{
    std_msgs::Float64 msg;
    msg.data = mu;

    _fc_pub.publish(msg);
}

void FrictionConeRosClient::setContactRotationMatrix(const Eigen::Matrix3d &R)
{
    geometry_msgs::Quaternion msg;
    tf::quaternionEigenToMsg(Eigen::Quaternion<double>(R), msg);

    _rot_pub.publish(msg);
}

CARTESIO_REGISTER_TASK_PLUGIN(FrictionConeImpl, FrictionCone)
CARTESIO_REGISTER_OPENSOT_CONSTR_PLUGIN(OpenSotFrictionConeAdapter, FrictionCone)
CARTESIO_REGISTER_ROS_CLIENT_API_PLUGIN(FrictionConeRosClient, FrictionCone)
CARTESIO_REGISTER_ROS_API_PLUGIN(FrictionConeRos, FrictionCone)



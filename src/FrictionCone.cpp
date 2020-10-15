#include "FrictionCone.h"

#include <geometry_msgs/QuaternionStamped.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/WrenchStamped.h>
#include <eigen_conversions/eigen_msg.h>
#include <visualization_msgs/Marker.h>

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

    _opensot_fc = boost::make_shared<FcSoT>(_ci_fc->getLinkName(),
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

FrictionConeRos::FrictionConeRos(TaskDescription::Ptr task,
                           RosContext::Ptr context):
    TaskRos(task, context)
{
    _ci_fc = std::dynamic_pointer_cast<FrictionCone>(task);
    if(!_ci_fc) throw std::runtime_error("Provided task description "
                                         "does not have expected type 'FrictionCone'");


    auto on_fc_rot_recv = [this](geometry_msgs::QuaternionStampedConstPtr msg)
    {
        Eigen::Quaterniond quat_r;
        quat_r.w() = msg->quaternion.w;
        quat_r.x() = msg->quaternion.x;
        quat_r.y() = msg->quaternion.y;
        quat_r.z() = msg->quaternion.z;

        _ci_fc->setContactRotationMatrix(quat_r.toRotationMatrix());
    };

    _fc_rot_sub = _ctx->nh().subscribe<geometry_msgs::QuaternionStamped>(task->getName() + "/rotation", 5,
                                                                         on_fc_rot_recv);

    auto on_fc_coeff_recv = [this](std_msgs::Float64ConstPtr msg)
    {
        double mu = msg->data;
        _ci_fc->setFrictionCoeff(mu);
    };

    _fc_coeff_sub = _ctx->nh().subscribe<std_msgs::Float64>(task->getName() + "/coeff_ref", 5,
                                                            on_fc_coeff_recv);


    _fc_rot_pub = _ctx->nh().advertise<geometry_msgs::QuaternionStamped>(task->getName() + "/contact_frame", 1);
    _fc_coeff_pub = _ctx->nh().advertise<std_msgs::Float64>(task->getName() + "/coeff_value", 1);

    _fc_viz = _ctx->nh().advertise<visualization_msgs::Marker>(task->getName() + "/marker", 1);

    /* Register type name */
    registerType("FrictionCone");
}

void FrictionConeRos::run(ros::Time time)
{
    TaskRos::run(time);

    /* publish mu */
    std_msgs::Float64 msg_mu;
    msg_mu.data = _ci_fc->getFrictionCoeff();
    _fc_coeff_pub.publish(msg_mu);

    /* publish rotation of contact frame */
    geometry_msgs::QuaternionStamped msg_quat;
    Eigen::Quaterniond quat;
    quat = Eigen::Quaterniond(_ci_fc->getContactFrame());

    tf::quaternionEigenToMsg(quat, msg_quat.quaternion);
    msg_quat.header.stamp = ros::Time::now();
    msg_quat.header.frame_id = "ci/" + _ci_fc->getLinkName();
    _fc_rot_pub.publish(msg_quat);


    /* publish friction cone */
    visualization_msgs::Marker marker;
    marker.points.clear();
    marker.ns = "friction_cone";
    marker.header.frame_id = "ci/" + _ci_fc->getLinkName();
    marker.header.stamp = ros::Time::now();
    marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
    marker.action = visualization_msgs::Marker::ADD;

    Eigen::Matrix3d c_R_fc;
    c_R_fc.setIdentity();

    if(_ci_fc->isLocal())
    {
        c_R_fc = _ci_fc->getContactFrame();
    }
    else
    {
        Eigen::Matrix3d w_R_c; w_R_c.setIdentity();
        _model->getOrientation(_ci_fc->getLinkName(), w_R_c);
        Eigen::Matrix3d w_R_fc = _ci_fc->getContactFrame();
        c_R_fc = w_R_c.transpose()*w_R_fc;
    }

    /* rotate it along the y axis to point in the z direction */
    Eigen::Matrix3d RotY; RotY.setIdentity();
    RotY(0,0) = std::cos(-M_PI_2); RotY(0,2) = std::sin(-M_PI_2);
    RotY(2,0) = -std::sin(-M_PI_2); RotY(2,2) = std::cos(-M_PI_2);

    c_R_fc = c_R_fc*RotY;


    Eigen::Quaterniond q(c_R_fc);
    marker.pose.orientation.x = q.x();
    marker.pose.orientation.y = q.y();
    marker.pose.orientation.z = q.z();
    marker.pose.orientation.w = q.w();


    geometry_msgs::Point pp[3];
    static const double DELTA_THETA = M_PI/16.0;
    double theta = 0.;
    double scale = 0.09;
    double magnitude = M_PI_2-std::atan(_ci_fc->getFrictionCoeff());
    for (std::size_t i = 0; i < 32; i++)
    {
       pp[0].x = 0;
       pp[0].y = 0;
       pp[0].z = 0;

       pp[1].x = scale;
       pp[1].y = scale * cos(theta) / magnitude;
       pp[1].z = scale * sin(theta) / magnitude;

       pp[2].x = scale;
       pp[2].y = scale * cos(theta + DELTA_THETA) / magnitude;
       pp[2].z = scale * sin(theta + DELTA_THETA) / magnitude;

       marker.points.push_back(pp[0]);
       marker.points.push_back(pp[1]);
       marker.points.push_back(pp[2]);

       theta += DELTA_THETA;
     }

    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;

    _fc_viz.publish(marker);
}

FrictionConeRosClient::FrictionConeRosClient(std::string name, ros::NodeHandle nh):
    TaskRos(name, nh)
{

    auto on_fc_cref_rcv = [this](std_msgs::Float64ConstPtr msg)
    {
        _mu_value = msg->data;
    };

    _fc_coeff_sub = _nh.subscribe<std_msgs::Float64>(name + "/coeff_value", 5, on_fc_cref_rcv);

    auto on_fc_rot_rcv = [this](geometry_msgs::QuaternionStampedConstPtr msg)
    {
        Eigen::Quaterniond cf_quat;
        cf_quat.w() = msg->quaternion.w;
        cf_quat.x() = msg->quaternion.x;
        cf_quat.y() = msg->quaternion.y;
        cf_quat.z() = msg->quaternion.z;

        _cf_rot = cf_quat.toRotationMatrix();
    };

    _fc_rot_sub = _nh.subscribe<geometry_msgs::QuaternionStamped>(name + "/contact_frame", 5, on_fc_rot_rcv);

    _fc_rot_pub = _nh.advertise<geometry_msgs::QuaternionStamped>(name + "/normal", 1);
    _fc_coeff_pub = _nh.advertise<std_msgs::Float64>(name + "/coeff", 1);
}


const std::string& FrictionConeRosClient::getLinkName() const
{
    throw std::runtime_error("Unsupported " + std::string() + __func__);
}

bool FrictionConeRosClient::isLocal() const
{
    throw std::runtime_error("Unsupported " + std::string() + __func__);
}

double FrictionConeRosClient::getFrictionCoeff() const
{
    return _mu_value;
}

void FrictionConeRosClient::setFrictionCoeff(const double mu)
{
    std_msgs::Float64 msg;
    msg.data = mu;
    _fc_coeff_pub.publish(msg);
}

Eigen::Matrix3d FrictionConeRosClient::getContactFrame() const
{
    return _cf_rot;
}

void FrictionConeRosClient::setContactRotationMatrix(const Eigen::Matrix3d &R)
{
    geometry_msgs::QuaternionStamped msg;
    Eigen::Quaterniond quat;
    quat = Eigen::Quaterniond(R);

    tf::quaternionEigenToMsg(quat, msg.quaternion);
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "ci/" + _link_name;
    _fc_rot_pub.publish(msg);
}

CARTESIO_REGISTER_TASK_PLUGIN(FrictionConeImpl, FrictionCone)
CARTESIO_REGISTER_OPENSOT_CONSTR_PLUGIN(OpenSotFrictionConeAdapter, FrictionCone)
CARTESIO_REGISTER_ROS_API_PLUGIN(FrictionConeRos, FrictionCone)
CARTESIO_REGISTER_ROS_CLIENT_API_PLUGIN(FrictionConeRosClient, FrictionCone)

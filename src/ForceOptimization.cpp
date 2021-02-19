#include "ForceOptimization.h"

ForceOptimizationClass::ForceOptimizationClass(std::string ns):
    _npr("~"),
    _nh(ns),
    _time(0.0),
    _active(false)
{
    load_params();
    load_model();
    load_ci();

    _timer = _nh.createTimer(ros::Duration(1./_rate),
                             &ForceOptimizationClass::on_timer_cb, this);

    _activation_srv = _nh.advertiseService("set_activation", &ForceOptimizationClass::activation_service, this);

    _timer.start();
}

void ForceOptimizationClass::load_params()
{
    std::string param;

    if(!_nh.searchParam("robot_description", param))
    {
        throw std::runtime_error("Missing parameter 'robot_description'");
    }
    else
    {
        ROS_INFO("Found URDF: %s", param.c_str());
        _nh.getParam(param, _urdf);
    }

    if(!_nh.searchParam("robot_description_semantic", param))
    {
        throw std::runtime_error("Missing parameter 'robot_description_semantic'");
    }
    else
    {
        ROS_INFO("Found SRDF: %s", param.c_str());
        _nh.getParam(param, _srdf);
    }

    if(!_npr.hasParam("problem_description"))
    {
        throw std::runtime_error("Missing parameter 'problem_description'");
    }
    else
    {
        _npr.getParam("problem_description", _fopt_pb_str);
    }

    _rate = _npr.param("rate", 100.0);
    _tf_prefix = _npr.param<std::string>("tf_prefix", "");
    _tf_prefix_slash = _tf_prefix + '/';
}

void ForceOptimizationClass::load_model()
{
    auto urdfdom = urdf::parseURDF(_urdf);

    ConfigOptions cfg;
    cfg.set_urdf(_urdf);
    cfg.set_srdf(_srdf);
    cfg.generate_jidmap();
    cfg.set_parameter("is_model_floating_base", is_model_floating_base(*urdfdom));
    cfg.set_parameter<std::string>("model_type", "RBDL");
    cfg.set_parameter<std::string>("framework", "ROS");

    _model = ModelInterface::getModel(cfg);
    _rspub = std::make_shared<XBot::Cartesian::Utils::RobotStatePublisher>(_model);

    Eigen::VectorXd q0;
    _model->getRobotState("home", q0);
    _model->setJointPosition(q0);
    _model->update();

    try
    {
        _robot = RobotInterface::getRobot(cfg);
        _robot->setControlMode(ControlMode::Effort() + ControlMode::Stiffness() + ControlMode::Damping());
        _imu = _robot->getImu().begin()->second;
    }
    catch(std::runtime_error& e)
    {

    }

    if(!_robot)
    {
        _js_sub = _nh.subscribe("joint_states", 1,
                                &ForceOptimizationClass::on_js_recv, this);
    }
    
    if(_npr.hasParam("torque_offset"))
    {
        auto torque_offset = _npr.param("torque_offset", std::map<std::string, double>());
        XBot::JointNameMap tau_off_map_xbot(torque_offset.begin(), torque_offset.end());
        _tau_offset.resize(_model->getJointNum());
        _model->mapToEigen(tau_off_map_xbot, _tau_offset);
    }
    else
        _tau_offset.resize(0);
}

void ForceOptimizationClass::load_ci()
{
    auto ctx = std::make_shared<Context>(
                std::make_shared<Parameters>(_rate),
                _model
                );

    ProblemDescription pb(YAML::Load(_fopt_pb_str), ctx);

    _ci = CartesianInterfaceImpl::MakeInstance("OpenSot",
                                               pb,
                                               ctx);

    RosServerClass::Options opt;
    opt.tf_prefix = _tf_prefix;
    opt.publish_tf = false;
    opt.ros_namespace = _nh.getNamespace();

    _rosapi = std::make_shared<RosServerClass>(_ci, opt);
}

bool ForceOptimizationClass::is_model_floating_base(const urdf::ModelInterface& urdf)
{
    auto link = urdf.root_link_;
    if(link->child_joints.size() == 1 &&
            link->child_joints[0]->type == urdf::Joint::FLOATING)
    {
        return true;
    }

    return false;
}

void ForceOptimizationClass::on_js_recv(sensor_msgs::JointStateConstPtr msg)
{
    for(int i = 0; i < msg->name.size(); i++)
    {
        _jmap[msg->name[i]] = msg->position.at(i);
    }

    _model->setJointPosition(_jmap);
}

void ForceOptimizationClass::on_timer_cb(const ros::TimerEvent&)
{
    Eigen::VectorXd tau;
    
    if(_robot)
    {
        _robot->sense(false);
        _model->syncFrom_no_update(*_robot, Sync::Position);
        _model->setFloatingBaseState(_imu);
    }

    _model->update();
    _rspub->publishTransforms(ros::Time::now(), "ci");

    if(!_ci->update(_time, 1./_rate))
    {
        ROS_WARN("Unable to solve");
        return;
    }

    _time += 1./_rate;

    _rosapi->run();
    
    std::cout << _tau_offset << std::endl;

    if(_robot && _active)
    {
        if (_tau_offset.size() > 0)
        {
            _model->getJointEffort(tau);
            tau += _tau_offset;
            _model->setJointEffort(tau);
        }
        _robot->setReferenceFrom(*_model, Sync::Effort);
        _robot->move();
    }

}

const CartesianInterfaceImpl::Ptr ForceOptimizationClass::getCartesianInterface() const
{
    return _ci;
}

bool ForceOptimizationClass::activation_service(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
    if (req.data == _active)
    {
        res.message = "You are setting the same activation state!";
        res.success = true;
        return res.success;
    }
    _active = req.data;
    res.message = "Activation state set!";
    res.success = true;
    return res.success;
}


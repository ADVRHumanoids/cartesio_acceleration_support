#include <ros/ros.h>
#include <urdf_parser/urdf_parser.h>
#include <sensor_msgs/JointState.h>

#include <XBotInterface/RobotInterface.h>

#include <cartesian_interface/CartesianInterfaceImpl.h>
#include <cartesian_interface/ros/RosServerClass.h>


using namespace XBot;
using namespace XBot::Cartesian;

class ForceOptimizationClass
{

public:

    ForceOptimizationClass(std::string ns);

private:

    static bool is_model_floating_base(const urdf::ModelInterface& urdf);

    void load_params();
    void load_model();
    void load_ci();

    void on_js_recv(sensor_msgs::JointStateConstPtr msg);
    void on_timer_cb(const ros::TimerEvent&);

    ros::NodeHandle _npr;
    ros::NodeHandle _nh;
    ros::Subscriber _js_sub;
    ros::Timer _timer;

    std::string _urdf, _srdf;
    std::string _fopt_pb_str;
    double _rate;
    std::string _tf_prefix, _tf_prefix_slash;

    ModelInterface::Ptr _model;
    RobotInterface::Ptr _robot;
    CartesianInterfaceImpl::Ptr _ci;
    RosServerClass::Ptr _rosapi;

    JointNameMap _jmap;
    double _time;

};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "force_optimization_node");

    ForceOptimizationClass fopt(ros::NodeHandle("~").param<std::string>("ns", "force_opt"));

    ros::spin();

    return 0;
}

ForceOptimizationClass::ForceOptimizationClass(std::string ns):
    _npr("~"),
    _nh(ns),
    _time(0.0)
{
    load_params();
    load_model();
    load_ci();

    _timer = _nh.createTimer(ros::Duration(1./_rate),
                             &ForceOptimizationClass::on_timer_cb, this);

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

    Eigen::VectorXd q0;
    _model->getRobotState("home", q0);
    _model->setJointPosition(q0);
    _model->update();

    try
    {
        _robot = RobotInterface::getRobot(cfg);
    }
    catch(std::runtime_error& e)
    {

    }

    if(!_robot)
    {
        _js_sub = _nh.subscribe("joint_states", 1,
                                &ForceOptimizationClass::on_js_recv, this);
    }

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

    if(_robot)
    {
        _robot->sense(false);
        _model->syncFrom_no_update(*_robot, Sync::Position);
    }

    _model->update();

    if(!_ci->update(_time, 1./_rate))
    {
        ROS_WARN("Unable to solve");
        return;
    }

    _time += 1./_rate;

    _rosapi->run();

    if(_robot)
    {
        _robot->setReferenceFrom(*_model, Sync::Effort);
        _robot->move();
    }

}

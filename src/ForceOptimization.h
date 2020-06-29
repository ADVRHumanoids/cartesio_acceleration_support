#ifndef FORCE_OPTIMIZATION_H
#define FORCE_OPTIMIZATION_H

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




#endif

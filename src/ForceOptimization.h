#ifndef FORCE_OPTIMIZATION_H
#define FORCE_OPTIMIZATION_H

#include <urdf_parser/urdf_parser.h>
#include <sensor_msgs/JointState.h>

#include <XBotInterface/RobotInterface.h>
#include <cartesian_interface/utils/RobotStatePublisher.h>

#include <cartesian_interface/CartesianInterfaceImpl.h>
#include <cartesian_interface/ros/RosServerClass.h>
#include <OpenSoT/tasks/Aggregated.h>

#include <std_srvs/SetBool.h>

using namespace XBot;
using namespace XBot::Cartesian;

class ForceOptimizationClass
{

public:

    ForceOptimizationClass(std::string ns);

    const CartesianInterfaceImpl::Ptr getCartesianInterface() const;

private:

    static bool is_model_floating_base(const urdf::ModelInterface& urdf);

    void load_params();
    void load_model();
    void load_ci();

    void on_js_recv(sensor_msgs::JointStateConstPtr msg);
    void on_timer_cb(const ros::TimerEvent&);

    bool activation_service(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);

    ros::NodeHandle _npr;
    ros::NodeHandle _nh;
    ros::Subscriber _js_sub;
    ros::Timer _timer;
    ros::ServiceServer _activation_srv;

    std::string _urdf, _srdf;
    std::string _fopt_pb_str;
    double _rate;
    std::string _tf_prefix, _tf_prefix_slash;

    ImuSensor::ConstPtr _imu;

    ModelInterface::Ptr _model;
    RobotInterface::Ptr _robot;
    CartesianInterfaceImpl::Ptr _ci;
    RosServerClass::Ptr _rosapi;
    
    Eigen::VectorXd _tau_offset;

    JointNameMap _jmap;
    double _time;
    bool _active;
    
    std::shared_ptr<XBot::Cartesian::Utils::RobotStatePublisher> _rspub;
};




#endif

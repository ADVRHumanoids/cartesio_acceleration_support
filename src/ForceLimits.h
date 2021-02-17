#ifndef FORCELIMITS_H
#define FORCELIMITS_H

#include <cartesian_interface/sdk/opensot/OpenSotTask.h>
#include <cartesian_interface/sdk/problem/Interaction.h>
#include <cartesian_interface/sdk/ros/server_api/TaskRos.h>
#include <cartesian_interface/sdk/ros/client_api/TaskRos.h>

#include <geometry_msgs/WrenchStamped.h>
#include <eigen_conversions/eigen_msg.h>

#include <OpenSoT/constraints/force/WrenchLimits.h>

using FlimSoT = OpenSoT::constraints::force::WrenchLimits;

namespace XBot { namespace Cartesian { namespace acceleration {

class ForceLimits : public virtual ConstraintDescription
{

public:

    CARTESIO_DECLARE_SMART_PTR(ForceLimits)

    virtual const std::string& getLinkName() const = 0;
    virtual bool isLocal() const = 0;
    virtual void getLimits(Eigen::Vector6d& fmin,
                           Eigen::Vector6d& fmax) const = 0;
    virtual void setLimits(const Eigen::Vector6d& fmin,
                           const Eigen::Vector6d& fmax) = 0;
    virtual void setZero() = 0;
    virtual void restore() = 0;

};

class ForceLimitsImpl : virtual public ForceLimits,
        public TaskDescriptionImpl
{

public:

    CARTESIO_DECLARE_SMART_PTR(ForceLimitsImpl)

    ForceLimitsImpl(YAML::Node node,
                    Context::ConstPtr context);

    const std::string& getLinkName() const override;
    bool isLocal() const override;
    void getLimits(Eigen::Vector6d& fmin, Eigen::Vector6d& fmax) const override;
    void setLimits(const Eigen::Vector6d& fmin, const Eigen::Vector6d& fmax) override;

    void setZero() override;
    void restore() override;

private:

    std::string _link;
    bool _local;
    Eigen::Vector6d _fmin, _fmax;
    bool _zeroed;

};

class OpenSotForceLimitsAdapter : public OpenSotConstraintAdapter
{

public:

    CARTESIO_DECLARE_SMART_PTR(OpenSotForceLimitsAdapter)

    OpenSotForceLimitsAdapter(ConstraintDescription::Ptr constr,
                              Context::ConstPtr);

    ConstraintPtr constructConstraint() override;

    OpenSoT::OptvarHelper::VariableVector getRequiredVariables() const override;

    void update(double time, double period) override;

private:

    ForceLimits::Ptr _ci_flim;
    FlimSoT::Ptr _opensot_flim;
    std::string _var_name;
};

class ForceLimitsRos : public ServerApi::TaskRos
{
public:


    CARTESIO_DECLARE_SMART_PTR(ForceLimitsRos)

    ForceLimitsRos(TaskDescription::Ptr task,
                   RosContext::Ptr context);

private:

    ForceLimits::Ptr _ci_force;
    ros::ServiceServer _toggle_srv;
    ros::Subscriber _flim_min_sub, _flim_max_sub;
    Eigen::Vector6d _flim_min, _flim_max;

};

class ForceLimitsRosClient : public ClientApi::TaskRos,
        virtual public ForceLimits
{
public:


    CARTESIO_DECLARE_SMART_PTR(ForceLimitsRosClient)

    ForceLimitsRosClient(std::string name,
                         ros::NodeHandle nh);

    const std::string& getLinkName() const override;
    void setLimits(const Eigen::Vector6d& fmin, const Eigen::Vector6d& fmax) override;
    void getLimits(Eigen::Vector6d& fmin, Eigen::Vector6d& fmax) const override;
    bool isLocal() const override;
    void setZero() override;
    void restore() override;

private:
    ros::Subscriber _flim_min_sub, _flim_max_sub;
    ros::Publisher _flim_min_pub, _flim_max_pub;
    Eigen::Vector6d _flim_min_value, _flim_max_value, _flim_min_ref, _flim_max_ref;
    std::string _link_name;
};

} } }
#endif // FORCELIMITS_H

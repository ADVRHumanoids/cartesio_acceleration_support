#ifndef CARTESIO_FRICTIONCONE_HPP
#define CARTESIO_FRICTIONCONE_HPP


#include <cartesian_interface/sdk/problem/Interaction.h>
#include <cartesian_interface/sdk/ros/server_api/TaskRos.h>
#include <cartesian_interface/sdk/ros/client_api/TaskRos.h>

#include <eigen_conversions/eigen_msg.h>
#include <OpenSoT/constraints/force/FrictionCone.h>

#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Float64.h>
#include <std_srvs/SetBool.h>

#include <cartesio_acceleration_support/FrictionCone.h>

using FcSoT = OpenSoT::constraints::force::FrictionCone;

namespace XBot { namespace Cartesian { namespace acceleration {

class FrictionConeImpl : virtual public FrictionCone,
        public TaskDescriptionImpl
{

public:

    CARTESIO_DECLARE_SMART_PTR(FrictionConeImpl)

    FrictionConeImpl(YAML::Node node,
                     Context::ConstPtr context);

    const std::string& getLinkName() const override;
    bool isLocal() const override;
    double getFrictionCoeff() const override;
    Eigen::Matrix3d getContactFrame() const override;
    void setContactRotationMatrix(const Eigen::Matrix3d& R) override;
    void setFrictionCoeff(const double mu) override;

private:

    std::string _link;
    bool _local;
    double _mu;
    Eigen::Matrix3d _R;
};

class OpenSotFrictionConeAdapter : public OpenSotConstraintAdapter
{

public:

    CARTESIO_DECLARE_SMART_PTR(OpenSotFrictionConeAdapter)

    OpenSotFrictionConeAdapter(ConstraintDescription::Ptr constr,
                               Context::ConstPtr);

    ConstraintPtr constructConstraint() override;

    OpenSoT::OptvarHelper::VariableVector getRequiredVariables() const override;

    void update(double time, double period) override;

private:

    FrictionCone::Ptr _ci_fc;
    FcSoT::Ptr _opensot_fc;
    std::string _var_name;
    /**
     * @brief _R used when is_local = true. It stores the last used rotation for the friction cone
     * to avoid computation of constraints even if not needed
     */
    Eigen::Matrix3d _R;
    double _mu;
};

class FrictionConeRos : public ServerApi::TaskRos
{
public:

    CARTESIO_DECLARE_SMART_PTR(FrictionConeRos)

    FrictionConeRos(TaskDescription::Ptr task,
                    RosContext::Ptr context);

    void run(ros::Time time) override;

private:
    FrictionCone::Ptr _ci_fc;

    ros::Subscriber _fc_sub, _rot_sub;
    ros::Publisher _fc_pub, _rot_pub;
};

class FrictionConeRosClient : public ClientApi::TaskRos,
        virtual public FrictionCone
{
public:

    CARTESIO_DECLARE_SMART_PTR(FrictionConeRosClient)

    FrictionConeRosClient(std::string name,
                          ros::NodeHandle nh);

    const std::string& getLinkName() const override;
    bool isLocal() const override;
    double getFrictionCoeff() const override;
    Eigen::Matrix3d getContactFrame() const override;
    void setContactRotationMatrix(const Eigen::Matrix3d& R) override;
    void setFrictionCoeff(const double mu) override;

private:
    std::string _link_name;
    double _fc;
    Eigen::Matrix3d _R;
    ros::Subscriber _rot_sub, _fc_sub;
    ros::Publisher _rot_pub, _fc_pub;
};

} } }


#endif // FRICTIONCONE_HPP

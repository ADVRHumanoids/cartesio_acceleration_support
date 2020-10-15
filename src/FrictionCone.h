#ifndef CARTESIO_FRICTIONCONE_H
#define CARTESIO_FRICTIONCONE_H


#include <cartesian_interface/sdk/opensot/OpenSotTask.h>
#include <cartesian_interface/sdk/problem/Interaction.h>
#include <cartesian_interface/sdk/ros/server_api/TaskRos.h>
#include <cartesian_interface/sdk/ros/client_api/TaskRos.h>

#include <OpenSoT/constraints/force/FrictionCone.h>

using FcSoT = OpenSoT::constraints::force::FrictionCone;

namespace XBot { namespace Cartesian { namespace acceleration {

class FrictionCone : public virtual ConstraintDescription
{

public:

    CARTESIO_DECLARE_SMART_PTR(FrictionCone)

    virtual const std::string& getLinkName() const = 0;
    virtual bool isLocal() const = 0;
    virtual double getFrictionCoeff() const = 0;
    virtual Eigen::Matrix3d getContactFrame() const = 0;
    /**
     * @brief setContactRotationMatrix
     * @param R is the rotation expressed in world frame if isLocal() = false, otherwise is the
     * local rotation applied to the rotation of the link_name expressed in world:
     *      w_R_link * R
     * NOTE: local = true is convenient for surface contacts
     */
    virtual void setContactRotationMatrix(const Eigen::Matrix3d& R) = 0;

    virtual void setFrictionCoeff(const double mu) = 0;
};

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
    ros::Subscriber _fc_rot_sub, _fc_coeff_sub;
    ros::Publisher _fc_rot_pub, _fc_coeff_pub, _fc_viz, _fc_porcodio;
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

    ros::Publisher _fc_rot_pub, _fc_coeff_pub;
    ros::Subscriber _fc_coeff_sub, _fc_rot_sub;
    double _mu_value;
    Eigen::Matrix3d _cf_rot;
    std::string _link_name;
};

} } }


#endif // FRICTIONCONE_H

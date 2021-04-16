#ifndef CARTESIO_NORMALTORQUE_H
#define CARTESIO_NORMALTORQUE_H


#include <cartesian_interface/sdk/opensot/OpenSotTask.h>
#include <cartesian_interface/sdk/problem/Interaction.h>
#include <cartesian_interface/sdk/ros/server_api/TaskRos.h>
#include <cartesian_interface/sdk/ros/client_api/TaskRos.h>

#include <OpenSoT/constraints/force/NormalTorque.h>

using NormalTorqueSoT = OpenSoT::constraints::force::NormalTorque;

namespace XBot { namespace Cartesian { namespace acceleration {

class NormalTorque : public virtual ConstraintDescription
{
public:

    CARTESIO_DECLARE_SMART_PTR(NormalTorque)

    virtual const std::string& getLinkName() const = 0;
    virtual const Eigen::Vector2d& getXLims() const = 0;
    virtual const Eigen::Vector2d& getYLims() const = 0;
    virtual double getFrictionCoeff() = 0;

};

class NormalTorqueImpl : virtual public NormalTorque,
        public TaskDescriptionImpl
{

public:

    CARTESIO_DECLARE_SMART_PTR(NormalTorqueImpl)

    NormalTorqueImpl(YAML::Node node, Context::ConstPtr context);

    const std::string& getLinkName() const override;
    const Eigen::Vector2d& getXLims() const override;
    const Eigen::Vector2d& getYLims() const override;
    double getFrictionCoeff() override;


private:

    std::string _link;
    Eigen::Vector2d _x_lim, _y_lim;
    double _mu;
};


class OpenSotNormalTorqueAdapter : public OpenSotConstraintAdapter
{

public:

    CARTESIO_DECLARE_SMART_PTR(OpenSotNormalTorqueAdapter)

    OpenSotNormalTorqueAdapter(ConstraintDescription::Ptr constr, Context::ConstPtr);

    ConstraintPtr constructConstraint() override;

    OpenSoT::OptvarHelper::VariableVector getRequiredVariables() const override;

    void update(double time, double period) override;

private:

    NormalTorque::Ptr _ci_nt;
    NormalTorqueSoT::Ptr _opensot_nt;
    std::string _var_name;
};

}}}


#endif

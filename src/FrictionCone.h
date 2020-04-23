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
};

} } }


#endif // FRICTIONCONE_H

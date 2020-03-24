#ifndef JOINTLIMITS_H
#define JOINTLIMITS_H

#include <cartesian_interface/sdk/problem/Limits.h>
#include <cartesian_interface/sdk/opensot/OpenSotTask.h>

#include <OpenSoT/constraints/acceleration/JointLimits.h>

using JointLimitsSoT = OpenSoT::constraints::acceleration::JointLimits;

namespace XBot { namespace Cartesian { namespace acceleration {

class JointLimitsAccImpl : public JointLimitsImpl
{
public:

    CARTESIO_DECLARE_SMART_PTR(JointLimitsAccImpl)

    JointLimitsAccImpl(YAML::Node task_node,
                       Context::ConstPtr context);

    const Eigen::VectorXd& getQddotMax() const;


private:

    Eigen::VectorXd _qddot_max;

};

class OpenSotJointLimitsAdapter :
        public OpenSotConstraintAdapter
{

public:

    OpenSotJointLimitsAdapter(ConstraintDescription::Ptr constr,
                              Context::ConstPtr context);

    OpenSoT::OptvarHelper::VariableVector getRequiredVariables() const override;

    virtual ConstraintPtr constructConstraint() override;

    virtual bool initialize(const OpenSoT::OptvarHelper& vars) override;

    virtual void update(double time, double period) override;

    virtual ~OpenSotJointLimitsAdapter() override = default;

protected:

private:

    JointLimitsSoT::Ptr _opensot_jlim;
    JointLimitsAccImpl::Ptr _ci_jlim;
};

}}}

#endif // JOINTLIMITS_H

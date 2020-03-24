#ifndef VELOCITYLIMITS_H
#define VELOCITYLIMITS_H


#include <boost/make_shared.hpp>

#include <cartesian_interface/problem/Limits.h>
#include <cartesian_interface/sdk/opensot/OpenSotTask.h>
#include <OpenSoT/constraints/acceleration/VelocityLimits.h>

using VelocityLimitsSoT = OpenSoT::constraints::acceleration::VelocityLimits;

namespace XBot { namespace Cartesian { namespace acceleration {

class OpenSotVelocityLimitsAdapter :
        public OpenSotConstraintAdapter
{

public:

    OpenSotVelocityLimitsAdapter(ConstraintDescription::Ptr constr,
                              Context::ConstPtr context);

    OpenSoT::OptvarHelper::VariableVector getRequiredVariables() const override;

    virtual ConstraintPtr constructConstraint() override;

    virtual bool initialize(const OpenSoT::OptvarHelper& vars) override;

    virtual void update(double time, double period) override;

    virtual ~OpenSotVelocityLimitsAdapter() override = default;

protected:

private:

    VelocityLimitsSoT::Ptr _opensot_vlim;
    VelocityLimits::Ptr _ci_vlim;
};




} } }

#endif // VELOCITYLIMITS_H

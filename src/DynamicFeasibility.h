#ifndef DYNAMICFEASIBILITY_H
#define DYNAMICFEASIBILITY_H


#include <cartesian_interface/sdk/problem/Constraint.h>
#include <cartesian_interface/sdk/opensot/OpenSotTask.h>

#include <OpenSoT/constraints/acceleration/DynamicFeasibility.h>

using DynamicFeasibilitySoT = OpenSoT::constraints::acceleration::DynamicFeasibility;

namespace XBot { namespace Cartesian { namespace acceleration {

class DynamicFeasibilityImpl : public ConstraintDescription
{
public:

    CARTESIO_DECLARE_SMART_PTR(DynamicFeasibilityImpl)

    DynamicFeasibilityImpl(YAML::Node task_node,
                           ModelInterface::ConstPtr model);

    std::vector<std::string> getContactLinks() const;


private:

    std::vector<std::string> _contact_links;

};

class OpenSotDynFeasAdapter :
        public OpenSotConstraintAdapter
{

public:

    OpenSotDynFeasAdapter(ConstraintDescription::Ptr constr,
                          ModelInterface::ConstPtr model);

    OpenSoT::OptvarHelper::VariableVector getRequiredVariables() const override;

    virtual ConstraintPtr constructConstraint() override;

    virtual bool initialize(const OpenSoT::OptvarHelper& vars) override;

    virtual void update(double time, double period) override;

    virtual ~OpenSotDynFeasAdapter() override = default;

protected:

private:

    DynamicFeasibilitySoT::Ptr _opensot_dynfeas;
    DynamicFeasibilityImpl::Ptr _ci_dynfeas;
};

}}}

#endif // DYNAMICFEASIBILITY_H

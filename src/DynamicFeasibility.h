#ifndef DYNAMICFEASIBILITY_H
#define DYNAMICFEASIBILITY_H


#include <cartesian_interface/sdk/problem/Constraint.h>
#include <cartesian_interface/sdk/opensot/OpenSotTask.h>

#include <OpenSoT/constraints/acceleration/DynamicFeasibility.h>

using DynamicFeasibilitySoT = OpenSoT::constraints::acceleration::DynamicFeasibility;

namespace XBot { namespace Cartesian { namespace acceleration {

class DynamicFeasibilityImpl : public TaskDescriptionImpl,
                               public virtual ConstraintDescription
{
public:

    CARTESIO_DECLARE_SMART_PTR(DynamicFeasibilityImpl)

    DynamicFeasibilityImpl(YAML::Node task_node,
                           Context::ConstPtr context);

    std::vector<std::string> getContactLinks() const;
    bool dynamicsEnabled() const;


private:

    bool _dynamics;
    std::vector<std::string> _contact_links;

};

class OpenSotDynFeasAdapter :
        public OpenSotConstraintAdapter
{

public:

    OpenSotDynFeasAdapter(ConstraintDescription::Ptr constr,
                          Context::ConstPtr context);

    OpenSoT::OptvarHelper::VariableVector getRequiredVariables() const override;

    virtual ConstraintPtr constructConstraint() override;

    virtual ~OpenSotDynFeasAdapter() override = default;

protected:

private:

    DynamicFeasibilitySoT::Ptr _opensot_dynfeas;
    DynamicFeasibilityImpl::Ptr _ci_dynfeas;
};

}}}

#endif // DYNAMICFEASIBILITY_H

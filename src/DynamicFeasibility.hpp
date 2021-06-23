#ifndef DYNAMICFEASIBILITY_HPP
#define DYNAMICFEASIBILITY_HPP

#include <OpenSoT/tasks/acceleration/DynamicFeasibility.h>
#include <cartesian_interface/sdk/opensot/OpenSotTask.h>

#include <cartesio_acceleration_support/DynamicFeasibility.h>

using DynamicFeasibilitySoT = OpenSoT::tasks::acceleration::DynamicFeasibility;

namespace XBot { namespace Cartesian { namespace acceleration {

class DynamicFeasibilityImpl : public TaskDescriptionImpl,
                               public virtual DynamicFeasibility
{
public:

    CARTESIO_DECLARE_SMART_PTR(DynamicFeasibilityImpl)

    DynamicFeasibilityImpl(YAML::Node task_node,
                           Context::ConstPtr context);

    std::vector<std::string> getContactLinks() const override;
    bool dynamicsEnabled() const override;


private:

    bool _dynamics;
    std::vector<std::string> _contact_links;

};

class OpenSotDynFeasAdapter :
        public OpenSotTaskAdapter
{

public:

    OpenSotDynFeasAdapter(TaskDescription::Ptr task,
                          Context::ConstPtr context);

    OpenSoT::OptvarHelper::VariableVector getRequiredVariables() const override;

    virtual TaskPtr constructTask() override;

    virtual ~OpenSotDynFeasAdapter() override = default;

protected:

private:

    DynamicFeasibilitySoT::Ptr _opensot_dynfeas;
    DynamicFeasibilityImpl::Ptr _ci_dynfeas;
};

}}}

#endif // DYNAMICFEASIBILITY_HPP

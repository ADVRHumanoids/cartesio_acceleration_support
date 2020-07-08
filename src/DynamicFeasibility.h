#ifndef DYNAMICFEASIBILITY_H
#define DYNAMICFEASIBILITY_H


#include <cartesian_interface/sdk/problem/Task.h>
#include <cartesian_interface/sdk/opensot/OpenSotTask.h>

#include <OpenSoT/tasks/acceleration/DynamicFeasibility.h>

using DynamicFeasibilitySoT = OpenSoT::tasks::acceleration::DynamicFeasibility;

namespace XBot { namespace Cartesian { namespace acceleration {

class DynamicFeasibilityImpl : public TaskDescriptionImpl
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

#endif // DYNAMICFEASIBILITY_H

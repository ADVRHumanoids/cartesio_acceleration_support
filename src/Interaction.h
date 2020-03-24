#ifndef INTERACTION_H
#define INTERACTION_H

#include <cartesian_interface/sdk/opensot/OpenSotTask.h>
#include <cartesian_interface/sdk/problem/Interaction.h>

#include <OpenSoT/tasks/acceleration/Cartesian.h>

using CartesianSoT = OpenSoT::tasks::acceleration::Cartesian;

namespace XBot { namespace Cartesian { namespace acceleration {



class OpenSotInteractionAdapter :
        public OpenSotTaskAdapter,
        public virtual CartesianTaskObserver
{

public:

    OpenSotInteractionAdapter(TaskDescription::Ptr ci_task,
                            Context::ConstPtr context);

    OpenSoT::OptvarHelper::VariableVector getRequiredVariables() const override;

    virtual TaskPtr constructTask() override;

    virtual bool initialize(const OpenSoT::OptvarHelper& vars) override;

    virtual void update(double time, double period) override;

    bool onBaseLinkChanged() override;

    bool onControlModeChanged() override;

    virtual ~OpenSotInteractionAdapter() override = default;

protected:

    CartesianSoT::Ptr _opensot_cart;

private:

    InteractionTask::Ptr _ci_cart;
    Eigen::VectorXd _x;

};

}}}

#endif // INTERACTION_H

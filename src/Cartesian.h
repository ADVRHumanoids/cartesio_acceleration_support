#ifndef CARTESIAN_H
#define CARTESIAN_H

#include <cartesian_interface/sdk/opensot/OpenSotTask.h>
#include <cartesian_interface/sdk/problem/Cartesian.h>

#include <OpenSoT/tasks/acceleration/Cartesian.h>

using CartesianSoT = OpenSoT::tasks::acceleration::Cartesian;

namespace XBot { namespace Cartesian { namespace acceleration {



class OpenSotCartesianAdapter :
        public OpenSotTaskAdapter,
        public virtual CartesianTaskObserver
{

public:

    OpenSotCartesianAdapter(TaskDescription::Ptr ci_task,
                            Context::ConstPtr context);

    OpenSoT::OptvarHelper::VariableVector getRequiredVariables() const override;

    virtual TaskPtr constructTask() override;

    virtual bool initialize(const OpenSoT::OptvarHelper& vars) override;

    virtual void update(double time, double period) override;

    bool onBaseLinkChanged() override;

    bool onControlModeChanged() override;

    virtual ~OpenSotCartesianAdapter() override = default;

protected:

    CartesianSoT::Ptr _opensot_cart;

private:

    CartesianTask::Ptr _ci_cart;
    double _old_lambda;

};

}}}

#endif // CARTESIAN_H

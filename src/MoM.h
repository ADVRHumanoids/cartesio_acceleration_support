#ifndef OPENSOTMOM_H
#define OPENSOTMOM_H

#include <boost/make_shared.hpp>

#include <cartesian_interface/AngularMomentum.h>
#include <cartesian_interface/problem/Cartesian.h>
#include <cartesian_interface/sdk/opensot/OpenSotTask.h>

#include <OpenSoT/tasks/acceleration/AngularMomentum.h>

using MomSoT = OpenSoT::tasks::acceleration::AngularMomentum;

namespace XBot { namespace Cartesian { namespace acceleration{

class OpenSotMomAdapter :
        public OpenSotTaskAdapter,
        public virtual CartesianTaskObserver
{

public:

    OpenSotMomAdapter(TaskDescription::Ptr task,
                      Context::ConstPtr context);

    OpenSoT::OptvarHelper::VariableVector getRequiredVariables() const override;

    virtual TaskPtr constructTask() override;

    virtual bool initialize(const OpenSoT::OptvarHelper& vars) override;

    virtual void update(double time, double period) override;

    bool onBaseLinkChanged() override;

    bool onControlModeChanged() override;

    virtual ~OpenSotMomAdapter() override = default;

protected:

private:

    MomSoT::Ptr _opensot_mom;
    AngularMomentum::Ptr _ci_mom;
    CartesianTask::Ptr _ci_cartesian;
    double _old_lambda;
};

} } }
#endif // OPENSOTMOM_H

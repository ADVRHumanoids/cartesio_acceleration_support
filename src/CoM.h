#ifndef OPENSOTCOM_H
#define OPENSOTCOM_H

#include <boost/make_shared.hpp>

#include <cartesian_interface/problem/Com.h>
#include <cartesian_interface/sdk/opensot/OpenSotTask.h>

#include <OpenSoT/tasks/acceleration/CoM.h>

using ComSoT = OpenSoT::tasks::acceleration::CoM;

namespace XBot { namespace Cartesian { namespace acceleration{

class OpenSotComAdapter :
        public OpenSotTaskAdapter,
        public virtual CartesianTaskObserver
{

public:

    OpenSotComAdapter(TaskDescription::Ptr task,
                      Context::ConstPtr context);

    OpenSoT::OptvarHelper::VariableVector getRequiredVariables() const override;

    virtual TaskPtr constructTask() override;

    virtual bool initialize(const OpenSoT::OptvarHelper& vars) override;

    virtual void update(double time, double period) override;

    bool onBaseLinkChanged() override;

    bool onControlModeChanged() override;

    virtual ~OpenSotComAdapter() override = default;

protected:

private:

    ComSoT::Ptr _opensot_com;
    CartesianTask::Ptr _ci_com;
    double _old_lambda;
};

} } }
#endif // OPENSOTCOM_H

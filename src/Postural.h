#ifndef POSTURAL_H
#define POSTURAL_H



#include <cartesian_interface/sdk/opensot/OpenSotTask.h>
#include <cartesian_interface/sdk/problem/Postural.h>

#include <OpenSoT/tasks/acceleration/Postural.h>

using PosturalSoT = OpenSoT::tasks::acceleration::Postural;

namespace XBot { namespace Cartesian { namespace acceleration {



class OpenSotPosturalAdapter :
        public OpenSotTaskAdapter,
        public virtual TaskObserver
{

public:

    OpenSotPosturalAdapter(TaskDescription::Ptr ci_task,
                            Context::ConstPtr context);

    OpenSoT::OptvarHelper::VariableVector getRequiredVariables() const override;

    virtual TaskPtr constructTask() override;

    virtual bool initialize(const OpenSoT::OptvarHelper& vars) override;

    virtual void update(double time, double period) override;

    virtual ~OpenSotPosturalAdapter() override = default;

protected:

    PosturalSoT::Ptr _opensot_post;

private:

    Eigen::VectorXd _qref;
    PosturalTask::Ptr _ci_post;

};

} } }

#endif // POSTURAL_H

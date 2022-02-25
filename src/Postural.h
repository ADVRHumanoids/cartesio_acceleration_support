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

    virtual const Eigen::MatrixXd& getOpenSotWeight() const override;

    Eigen::VectorXd _qref;
    PosturalTask::Ptr _ci_post;
    mutable Eigen::MatrixXd _W;

    Eigen::MatrixXd _B; // inertia matrix
    Eigen::MatrixXd _W_base; // auxiliary matrix used to get the value of "weight" tag in the .yaml file
    Eigen::MatrixXd _W_final; // final weight matrix (_W_base * _B)

};

} } }

#endif // POSTURAL_H

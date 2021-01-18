#ifndef POSTURAL_INTERACTION_H
#define POSTURAL_INTERACTION_H




#include "Postural.h"
#include <OpenSoT/tasks/acceleration/Postural.h>

using PosturalSoT = OpenSoT::tasks::acceleration::Postural;

namespace XBot { namespace Cartesian { namespace acceleration {

class PosturalInteractionTask : public virtual PosturalTask
{

public:

    CARTESIO_DECLARE_SMART_PTR(PosturalInteractionTask)

    virtual void setImpedance(const Eigen::VectorXd& K, const Eigen::VectorXd& D) = 0;
    virtual void getImpedance(Eigen::VectorXd& K, Eigen::VectorXd& D) const = 0;

};

class PosturalInteractionTaskImpl : public virtual PosturalInteractionTask,
                                    public PosturalTaskImpl
{

public:

    CARTESIO_DECLARE_SMART_PTR(PosturalInteractionTaskImpl)

    PosturalInteractionTaskImpl(YAML::Node node, Context::ConstPtr context);

    void setImpedance(const Eigen::VectorXd& K, const Eigen::VectorXd& D) override;
    void getImpedance(Eigen::VectorXd& K, Eigen::VectorXd& D) const override;

private:

    Eigen::VectorXd _K, _D;

};

class OpenSotPosturalInteractionAdapter :
        public OpenSotPosturalAdapter
{

public:

    OpenSotPosturalInteractionAdapter(TaskDescription::Ptr ci_task,
                        Context::ConstPtr context);

    virtual void update(double time, double period) override;

    virtual TaskPtr constructTask() override;

    virtual ~OpenSotPosturalInteractionAdapter() override = default;

private:
    PosturalInteractionTask::Ptr _ci_postural_interaction;

};

}
}
}

#endif

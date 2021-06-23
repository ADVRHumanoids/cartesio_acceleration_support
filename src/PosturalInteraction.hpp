#ifndef POSTURAL_INTERACTION_HPP
#define POSTURAL_INTERACTION_HPP

#include "Postural.h"
#include <cartesio_acceleration_support/PosturalInteraction.h>

using PosturalSoT = OpenSoT::tasks::acceleration::Postural;

namespace XBot { namespace Cartesian { namespace acceleration {

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

}}}

#endif // POSTURAL_INTERACTION_HPP

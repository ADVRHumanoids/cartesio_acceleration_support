#ifndef POSTURAL_INTERACTION_H
#define POSTURAL_INTERACTION_H

#include <cartesian_interface/sdk/opensot/OpenSotTask.h>

namespace XBot { namespace Cartesian { namespace acceleration {

class PosturalInteractionTask : public virtual PosturalTask
{

public:

    CARTESIO_DECLARE_SMART_PTR(PosturalInteractionTask)

    virtual void setImpedance(const Eigen::VectorXd& K, const Eigen::VectorXd& D) = 0;
    virtual void getImpedance(Eigen::VectorXd& K, Eigen::VectorXd& D) const = 0;

};

}}}

#endif // POSTURAL_INTERACTION_H

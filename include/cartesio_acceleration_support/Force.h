#ifndef FORCE_H
#define FORCE_H

#include <cartesian_interface/sdk/opensot/OpenSotTask.h>

namespace XBot { namespace Cartesian { namespace acceleration {

class ForceTask : public virtual TaskDescription
{

public:

    CARTESIO_DECLARE_SMART_PTR(ForceTask)

    virtual const std::string& getLinkName() const = 0;

    virtual const Eigen::Vector6d& getForceReference() const = 0;
    virtual const Eigen::Vector6d& getForceValue() const = 0;
    virtual Eigen::Affine3d getForceFrame() const = 0;

    virtual void setForceReference(const Eigen::Vector6d& f) = 0;
    virtual void setForceValue(const Eigen::Vector6d& f) = 0;
    virtual void setForceFrame(const Eigen::Affine3d& T) = 0;

};

}}}
#endif // FORCE_H

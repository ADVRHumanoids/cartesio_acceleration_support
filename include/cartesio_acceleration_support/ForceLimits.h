#ifndef FORCELIMITS_H
#define FORCELIMITS_H

#include <cartesian_interface/sdk/opensot/OpenSotTask.h>

namespace XBot { namespace Cartesian { namespace acceleration {

class ForceLimits : public virtual ConstraintDescription
{

public:

    CARTESIO_DECLARE_SMART_PTR(ForceLimits)

    virtual const std::string& getLinkName() const = 0;
    virtual bool isLocal() const = 0;
    virtual void getLimits(Eigen::Vector6d& fmin,
                           Eigen::Vector6d& fmax) const = 0;
    virtual void setLimits(const Eigen::Vector6d& fmin,
                           const Eigen::Vector6d& fmax) = 0;
    virtual void setZero() = 0;
    virtual void restore() = 0;

};

}}}

#endif // FORCELIMITS_H

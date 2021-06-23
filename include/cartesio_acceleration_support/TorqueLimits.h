#ifndef TORQUELIMITS_H
#define TORQUELIMITS_H

#include <cartesian_interface/sdk/opensot/OpenSotTask.h>

namespace XBot { namespace Cartesian { namespace acceleration {

class TorqueLimits : public virtual ConstraintDescription
{

public:

    CARTESIO_DECLARE_SMART_PTR(TorqueLimits)

    virtual const Eigen::VectorXd& getLimits() const = 0;
    virtual void setLimits(Eigen::VectorXd& tau_lims) = 0;
    virtual const std::vector<std::string>& getLinksInContact() const = 0;

};

}}}

#endif // TORQUELIMITS_H

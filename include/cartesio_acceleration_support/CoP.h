#ifndef CARTESIO_CENTEROFPRESSURE_H
#define CARTESIO_CENTEROFPRESSURE_H

#include <cartesian_interface/sdk/opensot/OpenSotTask.h>

namespace XBot { namespace Cartesian { namespace acceleration {

class CoP : public virtual ConstraintDescription
{
public:

    CARTESIO_DECLARE_SMART_PTR(CoP)

    virtual const std::string& getLinkName() const = 0;
    virtual const Eigen::Vector2d& getXLims() const = 0;
    virtual const Eigen::Vector2d& getYLims() const = 0;

};


}}}


#endif // CARTESIO_CENTEROFPRESSURE_H

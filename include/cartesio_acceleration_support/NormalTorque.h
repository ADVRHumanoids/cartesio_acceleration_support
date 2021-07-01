#ifndef CARTESIO_NORMALTORQUE_H
#define CARTESIO_NORMALTORQUE_H

#include <cartesian_interface/sdk/opensot/OpenSotTask.h>

namespace XBot { namespace Cartesian { namespace acceleration {

class NormalTorque : public virtual ConstraintDescription
{
public:

    CARTESIO_DECLARE_SMART_PTR(NormalTorque)

    virtual const std::string& getLinkName() const = 0;
    virtual const Eigen::Vector2d& getXLims() const = 0;
    virtual const Eigen::Vector2d& getYLims() const = 0;
    virtual double getFrictionCoeff() = 0;

};

}}}

#endif // CARTESIO_NORMALTORQUE_H

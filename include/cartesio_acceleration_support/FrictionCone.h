#ifndef CARTESIO_FRICTIONCONE_H
#define CARTESIO_FRICTIONCONE_H


#include <cartesian_interface/sdk/opensot/OpenSotTask.h>

namespace XBot { namespace Cartesian { namespace acceleration {

class FrictionCone : public virtual ConstraintDescription
{

public:

    CARTESIO_DECLARE_SMART_PTR(FrictionCone)

    virtual const std::string& getLinkName() const = 0;
    virtual bool isLocal() const = 0;
    virtual double getFrictionCoeff() const = 0;
    virtual Eigen::Matrix3d getContactFrame() const = 0;
    /**
     * @brief setContactRotationMatrix
     * @param R is the rotation expressed in world frame if isLocal() = false, otherwise is the
     * local rotation applied to the rotation of the link_name expressed in world:
     *      w_R_link * R
     * NOTE: local = true is convenient for surface contacts
     */
    virtual void setContactRotationMatrix(const Eigen::Matrix3d& R) = 0;

    virtual void setFrictionCoeff(const double mu) = 0;
};


}}}

#endif // FRICTIONCONE_H

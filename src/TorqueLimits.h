#ifndef TORQUELIMITS_H
#define TORQUELIMITS_H

#include <cartesian_interface/sdk/opensot/OpenSotTask.h>

#include <cartesian_interface/sdk/ros/server_api/TaskRos.h>
#include <cartesian_interface/sdk/ros/client_api/TaskRos.h>

#include <OpenSoT/constraints/acceleration/TorqueLimits.h>

using TaulimSoT = OpenSoT::constraints::acceleration::TorqueLimits;

namespace XBot { namespace Cartesian { namespace acceleration {

class TorqueLimits : public virtual ConstraintDescription
{

public:

    CARTESIO_DECLARE_SMART_PTR(TorqueLimits)

    virtual const Eigen::VectorXd& getLimits() const = 0;
    virtual void setLimits(Eigen::VectorXd& tau_lims) = 0;
    virtual const std::vector<std::string>& getLinksInContact() const = 0;

};

class TorqueLimitsImpl : virtual public TorqueLimits,
        public TaskDescriptionImpl
{

public:

    CARTESIO_DECLARE_SMART_PTR(TorqueLimitsImpl)

    TorqueLimitsImpl(YAML::Node node,
                    Context::ConstPtr context);

    const Eigen::VectorXd& getLimits() const override;
    void setLimits(Eigen::VectorXd& tau_lims) override;

    const std::vector<std::string>& getLinksInContact() const override;

private:

    Eigen::VectorXd _tau_lims;
    std::vector<std::string> _contact_links;
};

class OpenSotTorqueLimitsAdapter : public OpenSotConstraintAdapter
{

public:

    CARTESIO_DECLARE_SMART_PTR(OpenSotTorqueLimitsAdapter)

    OpenSotTorqueLimitsAdapter(ConstraintDescription::Ptr constr,
                              Context::ConstPtr context);

    ConstraintPtr constructConstraint() override;

    OpenSoT::OptvarHelper::VariableVector getRequiredVariables() const override;

    void update(double time, double period) override;

private:

    TorqueLimits::Ptr _ci_taulim;
    TaulimSoT::Ptr _opensot_taulim;

};

}
}
}

#endif

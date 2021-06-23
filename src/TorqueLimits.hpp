#ifndef TORQUELIMITS_HPP
#define TORQUELIMITS_HPP

#include <cartesian_interface/sdk/ros/server_api/TaskRos.h>
#include <cartesian_interface/sdk/ros/client_api/TaskRos.h>

#include <OpenSoT/constraints/acceleration/TorqueLimits.h>

#include <cartesio_acceleration_support/TorqueLimits.h>

using TaulimSoT = OpenSoT::constraints::acceleration::TorqueLimits;

namespace XBot { namespace Cartesian { namespace acceleration {

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

}}}

#endif // TORQUELIMITS_HPP

#ifndef CARTESIO_CENTEROFPRESSURE_HPP
#define CARTESIO_CENTEROFPRESSURE_HPP


#include <cartesian_interface/sdk/problem/Interaction.h>
#include <cartesian_interface/sdk/ros/server_api/TaskRos.h>
#include <cartesian_interface/sdk/ros/client_api/TaskRos.h>

#include <OpenSoT/constraints/force/CoP.h>

#include <cartesio_acceleration_support/CoP.h>

using CoPSoT = OpenSoT::constraints::force::CoP;

namespace XBot { namespace Cartesian { namespace acceleration {

class CoPImpl : virtual public CoP,
        public TaskDescriptionImpl
{

public:

    CARTESIO_DECLARE_SMART_PTR(CoPImpl)

    CoPImpl(YAML::Node node, Context::ConstPtr context);

    const std::string& getLinkName() const override;
    const Eigen::Vector2d& getXLims() const override;
    const Eigen::Vector2d& getYLims() const override;


private:

    std::string _link;
    Eigen::Vector2d _x_lim, _y_lim;
};

class OpenSotCoPAdapter : public OpenSotConstraintAdapter
{

public:

    CARTESIO_DECLARE_SMART_PTR(OpenSotCoPAdapter)

    OpenSotCoPAdapter(ConstraintDescription::Ptr constr, Context::ConstPtr);

    ConstraintPtr constructConstraint() override;

    OpenSoT::OptvarHelper::VariableVector getRequiredVariables() const override;

    void update(double time, double period) override;

private:

    CoP::Ptr _ci_cop;
    CoPSoT::Ptr _opensot_cop;
    std::string _var_name;
};

}}}


#endif // CARTESIO_CENTEROFPRESSURE_HPP

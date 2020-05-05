#ifndef CARTESIO_CENTEROFPRESSURE_H
#define CARTESIO_CENTEROFPRESSURE_H


#include <cartesian_interface/sdk/opensot/OpenSotTask.h>
#include <cartesian_interface/sdk/problem/Interaction.h>
#include <cartesian_interface/sdk/ros/server_api/TaskRos.h>
#include <cartesian_interface/sdk/ros/client_api/TaskRos.h>

#include <OpenSoT/constraints/force/CoP.h>

using CoPSoT = OpenSoT::constraints::force::CoP;

namespace XBot { namespace Cartesian { namespace acceleration {

class CoP : public virtual ConstraintDescription
{
public:

    CARTESIO_DECLARE_SMART_PTR(CoP)

    virtual const std::string& getLinkName() const = 0;
    virtual const Eigen::Vector2d& getXLims() const = 0;
    virtual const Eigen::Vector2d& getYLims() const = 0;

};

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


#endif

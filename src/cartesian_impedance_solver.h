#ifndef CARTESIAN_IMPEDANCE_SOLVER_H
#define CARTESIAN_IMPEDANCE_SOLVER_H

#include <cartesian_interface/CartesianInterfaceImpl.h>
#include <cartesian_interface/sdk/problem/Interaction.h>
#include <OpenSoT/tasks/torque/CartesianImpedanceCtrl.h>

namespace XBot { namespace Cartesian {


class CartesianImpedanceSolver : public CartesianInterfaceImpl
{

public:

    CartesianImpedanceSolver(ProblemDescription ik,
                             Context::Ptr ctx);

    bool update(double time, double period) override;

private:

    typedef OpenSoT::tasks::torque::CartesianImpedanceCtrl ImpedanceOpenSot;

    Eigen::VectorXd _tau;
    Eigen::MatrixXd _J;

    std::vector<InteractionTask::Ptr> _ci_tasks;
    std::vector<ImpedanceOpenSot::Ptr> _sot_tasks;


};

}}

#endif // CARTESIAN_IMPEDANCE_SOLVER_H

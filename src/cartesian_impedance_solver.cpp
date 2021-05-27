#include "cartesian_impedance_solver.h"
#include <cartesian_interface/sdk/SolverPlugin.h>
#include <cartesian_interface/sdk/opensot/OpenSotUtils.h>
#include <fmt/format.h>

using namespace XBot::Cartesian;


CartesianImpedanceSolver::CartesianImpedanceSolver(ProblemDescription ik,
                                                   Context::Ptr ctx):
    CartesianInterfaceImpl(ik, ctx)
{
    for(auto t : ik.getTask(static_cast<int>(0)))  // only support one prio
    {
        auto imp = std::dynamic_pointer_cast<InteractionTask>(t);

        if(!imp)
        {
            throw std::runtime_error(fmt::format(
                "task '{}' type unsupported; solver 'impedance' only"
                "supports interaction tasks", t->getName()));
        }

        auto imp_sot =  SotUtils::make_shared<ImpedanceOpenSot>(
            t->getName(),
            Eigen::VectorXd(),
            *_model,
            imp->getDistalLink(),
            imp->getBaseLink()
            );

        imp_sot->useInertiaMatrix(false);

        _ci_tasks.push_back(imp);

        _sot_tasks.push_back(imp_sot);

    }
}


bool CartesianImpedanceSolver::update(double time,
                                      double period)
{
    CartesianInterfaceImpl::update(time, period);

    _tau.setZero(_model->getJointNum());

    for(int i = 0; i < _ci_tasks.size(); i++)
    {
        auto imp_ci = _ci_tasks[i];
        auto sot_ci = _sot_tasks[i];

        static Eigen::MatrixXd kp(6, 6), kd(6, 6);

        kp = imp_ci->getStiffness();
        kd = imp_ci->getDamping();
        sot_ci->setStiffnessDamping(kp,
                                    kd);

        Eigen::Affine3d Tref;
        imp_ci->getPoseReference(Tref);
        sot_ci->setReference(Tref.matrix());

        sot_ci->update(Eigen::VectorXd());

        static Eigen::VectorXd fk(6), fd(6), f(6);
        sot_ci->getSpringForce(fk);
        sot_ci->getDamperForce(fd);

        f = fk + fd;

        _model->getJacobian(
            sot_ci->getDistalLink(),
            sot_ci->getBaseLink(),
            _J);

        _tau.noalias() += _J.transpose() * f;

    }

    _model->setJointEffort(_tau);

    return true;
}

CARTESIO_REGISTER_SOLVER_PLUGIN(CartesianImpedanceSolver, Impedance)

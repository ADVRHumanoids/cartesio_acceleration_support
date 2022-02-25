#include "Postural.h"

#include <boost/make_shared.hpp>

using namespace XBot::Cartesian::acceleration;

OpenSotPosturalAdapter::OpenSotPosturalAdapter(TaskDescription::Ptr task,
                                               Context::ConstPtr context):
    OpenSotTaskAdapter(task, context)
{
    _ci_post = std::dynamic_pointer_cast<PosturalTask>(task);

    if(!_ci_post) throw std::runtime_error("Provided task description "
                                           "does not have expected type 'PosturalTask'");


}

TaskPtr OpenSotPosturalAdapter::constructTask()
{
    Eigen::VectorXd q;
    _model->getJointPosition(q);

    _opensot_post = SotUtils::make_shared<PosturalSoT>(const_cast<ModelInterface&>(*_model),
                                                    _vars.getVariable("qddot"),
                                                    _ci_post->getName()
                                                    );


    return _opensot_post;
}

bool OpenSotPosturalAdapter::initialize(const OpenSoT::OptvarHelper& vars)
{
    return OpenSotTaskAdapter::initialize(vars);
}

void OpenSotPosturalAdapter::update(double time, double period)
{
    const double dt = _ctx->params()->getControlPeriod();

    if(_ci_post->getLambda2() >= 0)
    {
        _opensot_post->setLambda(_ci_post->getLambda()/dt/dt,
                                 _ci_post->getLambda2()/dt);
    }
    else
    {
        _opensot_post->setLambda(_ci_post->getLambda()/dt/dt);

    }

    /* Update reference */
    _ci_post->getReferencePosture(_qref);
    _opensot_post->setReference(_qref);

    if (_ci_post->useInertiaMatrixWeight())
    {
        _model->getInertiaMatrix(_B);
        _W_base = _ci_post->getWeight();
        _W_final = _W_base * _B;
        _opensot_post->setWeight(_W_final);
    }
}

const Eigen::MatrixXd& OpenSotPosturalAdapter::getOpenSotWeight() const
{
    int w_size = _opensot_post->getTaskSize();
    _W = _ci_task->getWeight().bottomRightCorner(w_size, w_size);
    return _W;
}

OpenSoT::OptvarHelper::VariableVector OpenSotPosturalAdapter::getRequiredVariables() const
{
    return {{"qddot", _model->getJointNum()}};
}

CARTESIO_REGISTER_OPENSOT_TASK_PLUGIN(OpenSotPosturalAdapter, Postural)



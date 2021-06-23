#include "PosturalInteraction.hpp"

using namespace XBot::Cartesian::acceleration;


PosturalInteractionTaskImpl::PosturalInteractionTaskImpl(YAML::Node node, Context::ConstPtr context):
    PosturalTaskImpl (node, context)
{
    _model->getStiffness(_K);
    _model->getDamping(_D);
}

void PosturalInteractionTaskImpl::setImpedance(const Eigen::VectorXd& K, const Eigen::VectorXd& D)
{
    _K = K;
    _D = D;
}

void PosturalInteractionTaskImpl::getImpedance(Eigen::VectorXd& K, Eigen::VectorXd& D) const
{
    K = _K;
    D = _D;
}

OpenSotPosturalInteractionAdapter::OpenSotPosturalInteractionAdapter(TaskDescription::Ptr ci_task,
                                                                     Context::ConstPtr context):
    OpenSotPosturalAdapter(ci_task, context)
{
    _ci_postural_interaction = std::dynamic_pointer_cast<PosturalInteractionTask>(ci_task);
    if(!_ci_postural_interaction) throw std::runtime_error("Provided task description "
                                            "does not have expected type 'PosturalInteractionTask'");


    double dt = context->params()->getControlPeriod();
    _ci_postural_interaction->setLambda(dt*dt);
    _ci_postural_interaction->setLambda2(dt);
}

void OpenSotPosturalInteractionAdapter::update(double time, double period)
{

    OpenSotPosturalAdapter::update(time, period);

    Eigen::VectorXd K, D;
    _ci_postural_interaction->getImpedance(K, D);
    _opensot_post->setGains(K.asDiagonal(), D.asDiagonal());
}

TaskPtr OpenSotPosturalInteractionAdapter::constructTask()
{
    OpenSotPosturalAdapter::constructTask();
    _opensot_post->setGainType(OpenSoT::tasks::acceleration::Force);

    return _opensot_post;
}

CARTESIO_REGISTER_TASK_PLUGIN(PosturalInteractionTaskImpl, PosturalInteraction)
CARTESIO_REGISTER_OPENSOT_TASK_PLUGIN(OpenSotPosturalInteractionAdapter, PosturalInteraction)

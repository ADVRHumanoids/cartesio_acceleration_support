#include "DynamicFeasibility.h"

using namespace XBot::Cartesian::acceleration;


OpenSotDynFeasAdapter::OpenSotDynFeasAdapter(ConstraintDescription::Ptr constr,
                                             ModelInterface::ConstPtr model):
    OpenSotConstraintAdapter (constr, model)
{

}

DynamicFeasibilityImpl::DynamicFeasibilityImpl(YAML::Node task_node,
                                               ModelInterface::ConstPtr model):
    ConstraintDescription(task_node, model)
{

}

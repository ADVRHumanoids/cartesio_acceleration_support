#include "FrictionCone.h"

using namespace XBot::Cartesian::acceleration;

namespace  {

std::string get_name(YAML::Node node)
{
    auto link = node["link"].as<std::string>();
    return "friction_cone_" + link;
}

}

FrictionConeImpl::FrictionConeImpl(YAML::Node node,
                                   Context::ConstPtr context):
    TaskDescriptionImpl(node, context, get_name(node), 5)
{
    _link = node["link"].as<std::string>();
    _local = false;
    _R.setIdentity();
    _mu = 1.0;

    if(auto loc = node["local"])
    {
        _local = loc.as<bool>();
    }

    if(auto mu = node["friction_coeff"])
    {
        _mu = mu.as<double>();
    }

    if(auto rot = node["contact_frame"])
    {
        auto quat = rot.as<std::vector<double>>();

        if(quat.size() != 4)
        {
            throw std::invalid_argument("Field 'contact_frame' must be a quaternion");
        }

        _R = Eigen::Quaterniond(Eigen::Vector4d::Map(quat.data())).toRotationMatrix();
    }

}


const std::string& FrictionConeImpl::getLinkName() const
{
    return _link;
}

bool FrictionConeImpl::isLocal() const
{
    return _local;
}

double FrictionConeImpl::getFrictionCoeff() const
{
    return _mu;
}

Eigen::Matrix3d FrictionConeImpl::getContactFrame() const
{
    return _R;
}



OpenSotFrictionConeAdapter::OpenSotFrictionConeAdapter(ConstraintDescription::Ptr constr,
                                                       Context::ConstPtr context):
    OpenSotConstraintAdapter(constr, context)
{
    _ci_fc = std::dynamic_pointer_cast<FrictionCone>(constr);
    if(!constr) throw std::runtime_error("Provided task description "
                                         "does not have expected type 'FrictionCone'");


    _var_name = "force_" + _ci_fc->getLinkName();
}

ConstraintPtr OpenSotFrictionConeAdapter::constructConstraint()
{
    FcSoT::friction_cone fc(_ci_fc->getContactFrame(), _ci_fc->getFrictionCoeff());

    _opensot_fc = boost::make_shared<FcSoT>(_ci_fc->getLinkName(),
                                            _vars.getVariable(_var_name),
                                            const_cast<ModelInterface&>(*_model),
                                            fc);

    return _opensot_fc;
}

OpenSoT::OptvarHelper::VariableVector OpenSotFrictionConeAdapter::getRequiredVariables() const
{
    return {{_var_name, 6}};
}

void OpenSotFrictionConeAdapter::update(double time, double period)
{
    OpenSotConstraintAdapter::update(time, period);

    if(_ci_fc->isLocal())
    {
        Eigen::Matrix3d w_R_link;
        _model->getOrientation(_ci_fc->getLinkName(), w_R_link);

        Eigen::Matrix3d link_R_c = _ci_fc->getContactFrame();

        _opensot_fc->setContactRotationMatrix(w_R_link * link_R_c);
    }
}

CARTESIO_REGISTER_TASK_PLUGIN(FrictionConeImpl, FrictionCone)
CARTESIO_REGISTER_OPENSOT_CONSTR_PLUGIN(OpenSotFrictionConeAdapter, FrictionCone)

#ifndef FORCE_H
#define FORCE_H


#include <cartesian_interface/sdk/opensot/OpenSotTask.h>
#include <cartesian_interface/sdk/problem/Interaction.h>
#include <cartesian_interface/sdk/ros/server_api/TaskRos.h>
#include <cartesian_interface/sdk/ros/client_api/TaskRos.h>

#include <OpenSoT/tasks/force/Force.h>

using WrenchSoT = OpenSoT::tasks::force::Wrench;

namespace XBot { namespace Cartesian { namespace acceleration {

class ForceTask : public virtual TaskDescription
{

public:

    CARTESIO_DECLARE_SMART_PTR(ForceTask)

    virtual const std::string& getLinkName() const = 0;

    virtual const Eigen::Vector6d& getForceReference() const = 0;
    virtual const Eigen::Vector6d& getForceValue() const = 0;
    virtual Eigen::Affine3d getForceFrame() const = 0;

    virtual void setForceReference(const Eigen::Vector6d& f) = 0;
    virtual void setForceValue(const Eigen::Vector6d& f) = 0;
    virtual void setForceFrame(const Eigen::Affine3d& T) = 0;

};


class ForceTaskImpl : public virtual ForceTask,
                      public TaskDescriptionImpl
{

public:

    CARTESIO_DECLARE_SMART_PTR(ForceTaskImpl)

    ForceTaskImpl(YAML::Node node, Context::ConstPtr context);

    const std::string& getLinkName() const override;
    const Eigen::Vector6d& getForceReference() const override;
    Eigen::Affine3d getForceFrame() const override;
    void setForceReference(const Eigen::Vector6d& f) override;
    const Eigen::Vector6d& getForceValue() const override;
    void setForceValue(const Eigen::Vector6d& f) override;
    void setForceFrame(const Eigen::Affine3d& T) override;

private:

    std::string _link;
    double _fref_timeout;
    Eigen::Vector6d _fref, _fvalue;
    Eigen::Affine3d _T;

};


class ForceTaskRos : public ServerApi::TaskRos
{

public:


    CARTESIO_DECLARE_SMART_PTR(ForceTaskRos)

    ForceTaskRos(TaskDescription::Ptr task,
                 RosContext::Ptr context);

    void run(ros::Time time) override;

private:

    ForceTask::Ptr _ci_force;
    ros::Publisher _f_pub;
    ros::Subscriber _fref_sub;
};

class ForceTaskRosClient : public ClientApi::TaskRos,
        virtual public ForceTask
{

public:


    CARTESIO_DECLARE_SMART_PTR(ForceTaskRosClient)

    ForceTaskRosClient(std::string name,
                       ros::NodeHandle nh);

    const std::string& getLinkName() const override;
    const Eigen::Vector6d& getForceReference() const override;
    const Eigen::Vector6d& getForceValue() const override;
    Eigen::Affine3d getForceFrame() const override;
    void setForceReference(const Eigen::Vector6d& f) override;
    void setForceValue(const Eigen::Vector6d& f) override;
    void setForceFrame(const Eigen::Affine3d& T) override;

private:


    ros::Subscriber _fvalue_sub;
    ros::Publisher _fref_pub;
    std::string _link_name;
    Eigen::Vector6d _fvalue, _fref;


};

class OpenSotForceAdapter :
        public OpenSotTaskAdapter
{

public:

    OpenSotForceAdapter(TaskDescription::Ptr ci_task,
                        Context::ConstPtr context);

    OpenSoT::OptvarHelper::VariableVector getRequiredVariables() const override;

    virtual TaskPtr constructTask() override;

    virtual void update(double time, double period) override;

    virtual void processSolution(const Eigen::VectorXd& solution) override;

    virtual ~OpenSotForceAdapter() override = default;

protected:

    WrenchSoT::Ptr _opensot_wrench;

private:

    OpenSoT::AffineHelper _var;
    std::string _var_name;
    ForceTask::Ptr _ci_force;
    Eigen::VectorXd _x;

};

}}}
#endif // FORCE_H

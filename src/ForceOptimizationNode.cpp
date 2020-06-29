#include <ros/ros.h>
#include <ForceOptimization.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "force_optimization_node");

    ForceOptimizationClass fopt(ros::NodeHandle("~").param<std::string>("ns", "force_opt"));

    ros::spin();

    return 0;
}










#include "ros/ros.h"
#include "bc_local_planner/bc_planner_ros.h"
#include "dynamic_reconfigure/server.h"
#include "bc_local_planner/bc_localConfig.h"

bc_local_planner::Parameter param;


void callback(bc_local_planner::bc_localConfig &config, uint32_t level)
{
    param.Is_running = config.Is_running;
    param.Is_training = config.Is_training;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "bc_planner_node");

    ros::NodeHandle node("~");

    dynamic_reconfigure::Server<bc_local_planner::bc_localConfig> server;
    dynamic_reconfigure::Server<bc_local_planner::bc_localConfig>::CallbackType f;

    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);


    bc_local_planner::BCPlannerROS bc_planner(node);

    ros::Rate rate(10);

    while(ros::ok())
    {
        bc_planner.cfgCallback(param);
        bc_planner.BCPlannerROScore();
        rate.sleep();
        ros::spinOnce();
    }
    return 0;
}


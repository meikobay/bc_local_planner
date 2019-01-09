#ifndef BC_LOCAL_PLANNER_BC_PLANNER_ROS_H_
#define BC_LOCAL_PLANNER_BC_PLANNER_ROS_H_
#include "ros/ros.h"
#include "string"
#include "iostream"
#include "tf/transform_listener.h"
#include "bc_local_planner/occupy_map.h"
#include "bc_local_planner/bc_planner_local.h"
#include "bc_local_planner/bc_planner_global.h"
#include "bc_local_planner/MCLearner.h"
#include "geometry_msgs/TwistStamped.h"

using std::cout;
using std::endl;


namespace bc_local_planner {


struct Parameter
{
    bool Is_running;
    bool Is_training;
};

class BCPlannerROS {
public:
    BCPlannerROS(ros::NodeHandle node);
    ~BCPlannerROS();

    void cfgCallback(Parameter &param);


    void BCPlannerROScore();
    void BCPlannerROSrun();
    void BCPlannerROStrain();

    void mapCallback(nav_msgs::OccupancyGrid map);
    void cwpCallback(geometry_msgs::PointStamped cwp);
    void cmdCallback(geometry_msgs::TwistStamped cmd);

    
private:
    bool bIsMapInitialed;
    bool bIsCWPInitialed;
    float runCurvatue_;
    float runVelocity_;

    vector<Data> dataSet_;

    Parameter param_;


    OccupyMap occupymap_;

    BCPlannerLocal localplanner_;
    BCPlannerGlobal globalplanner_;
    MCLearner mclearner_;

    geometry_msgs::Point ugvPose_;
    geometry_msgs::Point targetPose_;
    geometry_msgs::Point cwp_point_;

    tf::TransformListener listener_;

    ros::Publisher pub_CandidatePathTORVIZ;
    ros::Publisher pub_FeasiblePathTORVIZ;
    ros::Publisher pub_OptimalPathToRVIZ;
    ros::Publisher pub_CandidateRCLTORVIZ;
    ros::Publisher pub_OptimalRCLToRVIZ;
    ros::Publisher pub_localtarget;

    ros::Publisher pub_CMD;

    ros::Subscriber sub_map_;
    ros::Subscriber sub_cwp_;
    ros::Subscriber sub_CMD_;

    ros::NodeHandle rosNode_;
};

}
#endif

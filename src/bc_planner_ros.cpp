#include "bc_local_planner/bc_planner_ros.h"


namespace bc_local_planner {
BCPlannerROS::BCPlannerROS(ros::NodeHandle node):
    rosNode_(node),
    bIsMapInitialed(false),
    bIsCWPInitialed(false),
    runCurvatue_(0.0),
    runVelocity_(0.0)
{
    localplanner_.updateWeigths(0.45, 0.45, 0.1);
    globalplanner_.updateWeigths(0.25, 0.25, 0.5);
    mclearner_.setWeights(0.1,0.1,0.8);

    pub_CandidatePathTORVIZ = rosNode_.advertise<nav_msgs::Path>("CandidatePathToRVIZ", 1);
    pub_FeasiblePathTORVIZ = rosNode_.advertise<nav_msgs::Path>("FeasiblePathToRVIZ", 1);
    pub_OptimalPathToRVIZ = rosNode_.advertise<nav_msgs::Path>("OptimalPathToRVIZ", 1);
    pub_CandidateRCLTORVIZ = rosNode_.advertise<nav_msgs::Path>("CandidateRCLToRVIZ", 1);
    pub_OptimalRCLToRVIZ = rosNode_.advertise<nav_msgs::Path>("OptimalRCLToRVIZ", 1);
    pub_localtarget = rosNode_.advertise<geometry_msgs::PoseStamped>("localtarget", 1);
    pub_CMD = rosNode_.advertise<geometry_msgs::TwistStamped>("/CMD", 1);

    sub_map_ = rosNode_.subscribe("/costmap_2d/costmap/costmap", 1, &BCPlannerROS::mapCallback, this);
    sub_cwp_ = rosNode_.subscribe("/cwp", 1, &BCPlannerROS::cwpCallback, this);
    sub_CMD_ = rosNode_.subscribe("/CMD", 1, &BCPlannerROS::cmdCallback, this);
}


BCPlannerROS::~BCPlannerROS(){}


void BCPlannerROS::cfgCallback(Parameter &param)
{
    param_ = param;
}

void BCPlannerROS::BCPlannerROScore()
{
    if(bIsCWPInitialed && bIsMapInitialed)
    {
        //-----------get ugv pose-----------------//
        tf::StampedTransform orig_pose;
        try{
          listener_.lookupTransform("/map", "/base_link", ros::Time(0), orig_pose);
        }
        catch (tf::TransformException ex){
          ROS_ERROR("%s",ex.what());
          ros::Duration(1.0).sleep();
        }

        geometry_msgs::Quaternion ugv_q;
        ugv_q.x = orig_pose.getRotation().getX();
        ugv_q.y = orig_pose.getRotation().getY();
        ugv_q.z = orig_pose.getRotation().getZ();
        ugv_q.w = orig_pose.getRotation().getW();
        ugvPose_.z  = tf::getYaw(ugv_q);

        ugvPose_.x = orig_pose.getOrigin().x();
        ugvPose_.y = orig_pose.getOrigin().y();

        // ----------end --------------//


        //-----------get local target-------//

        globalplanner_.genearteCandidate(ugvPose_, cwp_point_);
        targetPose_ = globalplanner_.getCurrentLG();
        nav_msgs::Path candidatercl2RVIZ = globalplanner_.getCandidateRCLToRVIZ();
        candidatercl2RVIZ.header.frame_id = "/map";
        candidatercl2RVIZ.header.stamp = ros::Time::now();
        pub_CandidateRCLTORVIZ.publish(candidatercl2RVIZ);

        nav_msgs::Path optimalrcl2RVIZ = globalplanner_.getOptimalRCLToRVIZ();
        optimalrcl2RVIZ.header.frame_id = "/map";
        optimalrcl2RVIZ.header.stamp = ros::Time::now();
        pub_OptimalRCLToRVIZ.publish(optimalrcl2RVIZ);

        geometry_msgs::PoseStamped local_target;
        local_target.header.frame_id = "/map";
        local_target.header.stamp = ros::Time::now();
        local_target.pose.position.x = targetPose_.x;
        local_target.pose.position.y = targetPose_.y;

        tf2::Quaternion lg_q;
        lg_q.setRPY(0.0, 0.0, targetPose_.z); // remap point.z to yaw

        local_target.pose.orientation.x = lg_q.x();
        local_target.pose.orientation.y = lg_q.y();
        local_target.pose.orientation.z = lg_q.z();
        local_target.pose.orientation.w = lg_q.w();
        pub_localtarget.publish(local_target);

        //----------------end--------------//

        //---------get curvature----//


        localplanner_.genearteCandidate(ugvPose_, targetPose_);

        nav_msgs::Path candidatepath2RVIZ = localplanner_.getCandidatePathToRVIZ();
        candidatepath2RVIZ.header.frame_id = "/map";
        candidatepath2RVIZ.header.stamp = ros::Time::now();
        pub_CandidatePathTORVIZ.publish(candidatepath2RVIZ);


        nav_msgs::Path feasiblepath2RVIZ = localplanner_.getFeasiblePathToRVIZ();
        feasiblepath2RVIZ.header.frame_id = "/map";
        feasiblepath2RVIZ.header.stamp = ros::Time::now();
        pub_FeasiblePathTORVIZ.publish(feasiblepath2RVIZ);

        if(param_.Is_running)
        {
            BCPlannerROSrun();
        }

        if(param_.Is_training)
        {
            BCPlannerROStrain();
        }
    }
}


void BCPlannerROS::BCPlannerROSrun()
{
    float currentCurvature = 0.0;
    float currentVelocity = 0.0;

    if(localplanner_.getCurrentCurvature(currentCurvature))
    {
        cout<<"current optimal curvature is :"<<currentCurvature<<endl;
        nav_msgs::Path optimalpath2RVIZ = localplanner_.getOptimalPathToRVIZ();
        optimalpath2RVIZ.header.frame_id = "/map";
        optimalpath2RVIZ.header.stamp = ros::Time::now();
        pub_OptimalPathToRVIZ.publish(optimalpath2RVIZ);


        //---------------get velocity-------------//
         float vcost = abs(currentCurvature);

         if(vcost <= 0.01)
             currentVelocity = 2.5;
         else if(vcost >= 0.01 && vcost <= 0.1)
             currentVelocity = 16.67*pow(vcost, 2)-18.5*vcost+2.68;
         else
             currentVelocity = 1.0;

        //------------------end------------------//

    }

   //-------------------end------------------//

    runCurvatue_ = currentCurvature;
    runVelocity_ = currentVelocity;

   //---------------pub cmd-------------------//
    geometry_msgs::TwistStamped cmd;
    cmd.header.frame_id = "/map";
    cmd.header.stamp = ros::Time::now();
    cmd.twist.linear.x = runVelocity_;
    cmd.twist.angular.z = runCurvatue_;
    pub_CMD.publish(cmd);
   //------------------end-------------------//
}

void BCPlannerROS::BCPlannerROStrain()
{
    dataSet_.push_back(localplanner_.getTrainData(runCurvatue_));
    mclearner_.feedLearner(dataSet_);
}

void BCPlannerROS::mapCallback(nav_msgs::OccupancyGrid map)
{
    if (!bIsMapInitialed) bIsMapInitialed = true;

    occupymap_.updateMap(map);
    localplanner_.updateOccupyMap(occupymap_);
    globalplanner_.updateOccupyMap(occupymap_);
}



void BCPlannerROS::cwpCallback(geometry_msgs::PointStamped cwp)
{
    if(!bIsCWPInitialed) bIsCWPInitialed = true;

    cwp_point_ = cwp.point;
}

void BCPlannerROS::cmdCallback(geometry_msgs::TwistStamped cmd)
{
    runCurvatue_ = cmd.twist.angular.z;
    runVelocity_ = cmd.twist.linear.x;
}

}




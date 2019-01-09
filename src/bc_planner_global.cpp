#include "bc_local_planner/bc_planner_global.h"

namespace bc_local_planner {
BCPlannerGlobal::BCPlannerGlobal():
    nAngleSampleNum_(90),
    fAngleSampleInterval_(M_PI/180.0),
    nSampleNum_(2*nAngleSampleNum_+1),
    fLookHeadDist_(15.0),
    fP1Dist_(2.63),
    fLGDist_(10.5)
{
    InitialData();
}

BCPlannerGlobal::~BCPlannerGlobal(){}


void BCPlannerGlobal::InitialData()
{
    CandidateRCLs_ = new BiBezierRCL[nSampleNum_];
}

geometry_msgs::Point BCPlannerGlobal::getCurrentLG()   //remap point.z to yaw
{
    float score_min = 1000.0;
    int optimal_index = -1;
    for(int i=0; i<nSampleNum_; i++)
    {
        if(CandidateRCLs_[i].score < score_min)
        {
            score_min = CandidateRCLs_[i].score;
            optimal_index = i;
        }
    }

    OptimalRCL_.poses.clear();
    geometry_msgs::Point P0 = CandidateRCLs_[optimal_index].P0;
    geometry_msgs::Point P1 = CandidateRCLs_[optimal_index].P1;
    geometry_msgs::Point P2 = CandidateRCLs_[optimal_index].P2;


    float ft = 0.0;
    float fDiffX = 0.0;
    float fDiffY = 0.0;
    float fRelativeHeading = 0.0;
    for(int j=0; j<20; j++)
    {
        ft = j*0.05;
        geometry_msgs::PoseStamped pose;
        /** get point */
        pose.pose.position.x = (1-ft)*(1-ft)*P0.x+2*(1-ft)*ft*P1.x+ft*ft*P2.x;
        pose.pose.position.y = (1-ft)*(1-ft)*P0.y+2*(1-ft)*ft*P1.y+ft*ft*P2.y;
        fDiffX = 2*((P1.x-P0.x)+(P2.x-2*P1.x+P0.x)*ft);
        fDiffY = 2*((P1.y-P0.y)+(P2.y-2*P1.y+P0.y)*ft);

        fRelativeHeading = atan2(fDiffY, fDiffX);

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, fRelativeHeading);

        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();

        OptimalRCL_.poses.push_back(pose);
    }

    geometry_msgs::Point lg_point;
    float cum_dist = 0;
    for(int i=0; i<OptimalRCL_.poses.size()-1; i++)
    {
        geometry_msgs::PoseStamped pose1 = OptimalRCL_.poses[i];
        geometry_msgs::PoseStamped pose2 = OptimalRCL_.poses[i+1];

        lg_point.x = pose1.pose.position.x;
        lg_point.y = pose1.pose.position.y;
        lg_point.z = tf::getYaw(pose1.pose.orientation);


        float distX = pose2.pose.position.x - pose1.pose.position.x;
        float distY = pose2.pose.position.y - pose1.pose.position.y;
        float dist = sqrt(pow(distX,2)+pow(distY,2));
        cum_dist = cum_dist + dist;
        if(cum_dist >= fLGDist_)
        {
            break;
        }
    }

    return lg_point;
}

void BCPlannerGlobal::genearteCandidate(geometry_msgs::Point ugv_pose, geometry_msgs::Point cwp_point)
{
    /// 1. sample targets ///
    int index = 0;
    for(int i=-nAngleSampleNum_; i<=nAngleSampleNum_; i++)
    {
        CandidateRCLs_[index].P0.x = ugv_pose.x;
        CandidateRCLs_[index].P0.y = ugv_pose.y;

        CandidateRCLs_[index].P1.x = ugv_pose.x + fP1Dist_*cos(ugv_pose.z);
        CandidateRCLs_[index].P1.y = ugv_pose.y + fP1Dist_*sin(ugv_pose.z);

        CandidateRCLs_[index].P2.x = ugv_pose.x + fLookHeadDist_*cos(ugv_pose.z+i*fAngleSampleInterval_);
        CandidateRCLs_[index].P2.y = ugv_pose.y + fLookHeadDist_*sin(ugv_pose.z+i*fAngleSampleInterval_);

        index++;
    }


    /// 2. score every candidate trajectories with three critera ///
    /// reset data ///
    max_smooth_ = -200.0;
    min_smooth_ = 1000.0;
    max_safe_ = -200.0;
    min_safe_ = 1000.0;
    max_target_ = -200.0;
    min_target_ = 1000.0;
    for(int i=0; i<nSampleNum_; i++)
    {
        evaluateTrajectory(i, cwp_point);
    }

    /// normalize the cost of three criteria ///
    for(int i=0; i<nSampleNum_; i++)
    {
        float smoothness = 0.5;
        if(max_smooth_ != min_smooth_)
            smoothness = (CandidateRCLs_[i].smoothness - min_smooth_) / (max_smooth_ - min_smooth_);
        CandidateRCLs_[i].smoothness = smoothness;

        float safeness = 0.5;
        if(max_safe_ != min_safe_)
            safeness = (CandidateRCLs_[i].safeness - min_safe_) / (max_safe_ - min_safe_);
        CandidateRCLs_[i].safeness = safeness;

        float target = 0.5;
        if(max_target_ != min_target_)
            target = (CandidateRCLs_[i].target- min_target_) / (max_target_ - min_target_);
        CandidateRCLs_[i].target = target;

        float score = wr_*safeness + ws_*smoothness + wt_*target;
        CandidateRCLs_[i].score = score;
    }
}


void BCPlannerGlobal::evaluateTrajectory(int index, geometry_msgs::Point cwp_point)
{
    float fTInterval = 0.05;
    float ft = 0.0;

    float cum_obs = 0.0;
    int num_obs = 1;

    int cellData = 0;

    while(ft <= 1.0)
    {
        if(ft>=1.0)
        {
            ft = 1.0;
            evaluatePoint(index, ft, cellData);
            ft = 1.2;
        }
        else
        {
            evaluatePoint(index, ft, cellData);
        }


        if(cellData >= 0)
        {
            cum_obs += cellData;
            num_obs++;
        }

        ft = ft + fTInterval;
    }

    CandidateRCLs_[index].smoothness = fabs(float(index)-float(nAngleSampleNum_));
    CandidateRCLs_[index].safeness = cum_obs/num_obs;
    CandidateRCLs_[index].target = sqrt(pow((cwp_point.x - CandidateRCLs_[index].P2.x), 2) +
                                        pow((cwp_point.y - CandidateRCLs_[index].P2.y), 2));

    if(max_smooth_ < CandidateRCLs_[index].smoothness)
        max_smooth_ = CandidateRCLs_[index].smoothness;
    if(min_smooth_ > CandidateRCLs_[index].smoothness)
        min_smooth_ = CandidateRCLs_[index].smoothness;

    if(max_safe_ < CandidateRCLs_[index].safeness)
        max_safe_ = CandidateRCLs_[index].safeness;
    if(min_safe_ > CandidateRCLs_[index].safeness)
        min_safe_ = CandidateRCLs_[index].safeness;

    if(max_target_ < CandidateRCLs_[index].target)
        max_target_ = CandidateRCLs_[index].target;
    if(min_target_ > CandidateRCLs_[index].target)
        min_target_ = CandidateRCLs_[index].target;

}

void BCPlannerGlobal::evaluatePoint(int index, float ft, int& cellData)
{
    geometry_msgs::Point P0 = CandidateRCLs_[index].P0;
    geometry_msgs::Point P1 = CandidateRCLs_[index].P1;
    geometry_msgs::Point P2 = CandidateRCLs_[index].P2;

    /** get point */
    float fPX = (1-ft)*(1-ft)*P0.x+2*(1-ft)*ft*P1.x+ft*ft*P2.x;
    float fPY = (1-ft)*(1-ft)*P0.y+2*(1-ft)*ft*P1.y+ft*ft*P2.y;


    /** get occupancy cell data */

    if (!occupymap_.getMapData(fPX, fPY, cellData))
    {
        cellData = -1;
    }
}


nav_msgs::Path BCPlannerGlobal::getCandidateRCLToRVIZ()
{
    nav_msgs::Path CandidateRCLToRVIZ;
    for(int i=0; i<nSampleNum_; i++)
    {
        geometry_msgs::Point P0 = CandidateRCLs_[i].P0;
        geometry_msgs::Point P1 = CandidateRCLs_[i].P1;
        geometry_msgs::Point P2 = CandidateRCLs_[i].P2;


        float ft = 0.0;
        for(int j=0; j<20; j++)
        {
            ft = j*0.05;
            geometry_msgs::PoseStamped pose;
            /** get point */
            pose.pose.position.x = (1-ft)*(1-ft)*P0.x+2*(1-ft)*ft*P1.x+ft*ft*P2.x;
            pose.pose.position.y = (1-ft)*(1-ft)*P0.y+2*(1-ft)*ft*P1.y+ft*ft*P2.y;
            CandidateRCLToRVIZ.poses.push_back(pose);
        }

        for(int j=20; j>=0; j--)
        {
            ft = j*0.05;
            geometry_msgs::PoseStamped pose;
            /** get point */
            pose.pose.position.x = (1-ft)*(1-ft)*P0.x+2*(1-ft)*ft*P1.x+ft*ft*P2.x;
            pose.pose.position.y = (1-ft)*(1-ft)*P0.y+2*(1-ft)*ft*P1.y+ft*ft*P2.y;
            CandidateRCLToRVIZ.poses.push_back(pose);
        }
    }
    return CandidateRCLToRVIZ;
}

nav_msgs::Path BCPlannerGlobal::getOptimalRCLToRVIZ()
{
    return OptimalRCL_;
}

}

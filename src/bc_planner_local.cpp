#include "bc_local_planner/bc_planner_local.h"

namespace bc_local_planner {
BCPlannerLocal::BCPlannerLocal():
    nDistSampleNum_(12),
    fDistSampleInterval_(0.35),
    nSampleNum_(2*nDistSampleNum_+1),
    nCandidateNum_(3*nSampleNum_),
    lastCurvature_(0.0),
    nOptimalIndex_(0)
{
    InitialData();
}

BCPlannerLocal::~BCPlannerLocal(){}


void BCPlannerLocal::InitialData()
{
    DistP0P1_ = new float[3];
    DistP2P3_ = new float[3];

    TargetSamples_ = new TargetPoint[nSampleNum_];
    CandidateTrajectories_ = new CubicBezierTrajectory[nCandidateNum_];
}

bool BCPlannerLocal::getCurrentCurvature(float &curvature)
{
    float score_min = 100.0;
    int optimal_index = -1;
    for(int i=0; i<nCandidateNum_; i++)
    {
        if(CandidateTrajectories_[i].score < score_min)
        {
            score_min = CandidateTrajectories_[i].score;
            optimal_index = i;
        }
    }

    if(optimal_index == -1)
    {
        return false;
    }
    else
    {
        curvature = CandidateTrajectories_[optimal_index].c0;
        lastCurvature_ = CandidateTrajectories_[optimal_index].c0;
        nOptimalIndex_ = optimal_index;
        return true;
    }

}

void BCPlannerLocal::genearteCandidate(geometry_msgs::Point ugv_pose, geometry_msgs::Point target_pose)
{
    float dist2target = sqrt((ugv_pose.x - target_pose.x)*(ugv_pose.x - target_pose.x) +
                             (ugv_pose.y - target_pose.y)*(ugv_pose.y - target_pose.y));
    for(int i=0;i<3;i++)
    {
        DistP0P1_[i] = dist2target*(i+1)/3.0;
        DistP2P3_[i] = dist2target*(i+1)/3.0;
    }

    /// 1. sample targets ///
    int index = 0;
    for(int i=-nDistSampleNum_; i<=nDistSampleNum_; i++)
    {
        TargetSamples_[index].fX = target_pose.x + i*fDistSampleInterval_*sin(target_pose.z);
        TargetSamples_[index].fY = target_pose.y - i*fDistSampleInterval_*cos(target_pose.z);
        TargetSamples_[index].fHeading = target_pose.z;
        index++;
    }

    /// 2. sample candidate trajectories ///
    for(int i=0; i<nSampleNum_; i++)
    {
        for(int j=0; j<3; j++)
        {
            CandidateTrajectories_[3*i+j].P0.x = ugv_pose.x;
            CandidateTrajectories_[3*i+j].P0.y = ugv_pose.y;

            CandidateTrajectories_[3*i+j].P1.x = ugv_pose.x + DistP0P1_[j]*cos(ugv_pose.z);
            CandidateTrajectories_[3*i+j].P1.y = ugv_pose.y + DistP0P1_[j]*sin(ugv_pose.z);

            CandidateTrajectories_[3*i+j].P2.x = TargetSamples_[i].fX - DistP2P3_[j]*cos(TargetSamples_[i].fHeading);
            CandidateTrajectories_[3*i+j].P2.y = TargetSamples_[i].fY - DistP2P3_[j]*sin(TargetSamples_[i].fHeading);

            CandidateTrajectories_[3*i+j].P3.x = TargetSamples_[i].fX;
            CandidateTrajectories_[3*i+j].P3.y = TargetSamples_[i].fY;
        }
    }


    /// 3. score every candidate trajectories with three critera ///
    /// reset data ///
    max_smooth_ = -200.0;
    min_smooth_ = 1000.0;
    max_safe_ = -200.0;
    min_safe_ = 1000.0;
    max_consist_ = -200.0;
    min_consist_ = 1000.0;
    for(int i=0; i<nCandidateNum_; i++)
    {
        evaluateTrajectory(i);
    }

    ///4. normalize the cost of three criteria ///
    for(int i=0; i<nCandidateNum_; i++)
    {
        if(CandidateTrajectories_[i].bFeasible == false || CandidateTrajectories_[i].bObstacle == true)
            CandidateTrajectories_[i].score = 200.0;
        else
        {
            float smoothness = 0.5;
            if(max_smooth_ != min_smooth_)
                smoothness = (CandidateTrajectories_[i].smoothness - min_smooth_) / (max_smooth_ - min_smooth_);
            CandidateTrajectories_[i].smoothness = smoothness;

            float safeness = 0.5;
            if(max_safe_ != min_safe_)
                safeness = (CandidateTrajectories_[i].safeness - min_safe_) / (max_safe_ - min_safe_);
            CandidateTrajectories_[i].safeness = safeness;

            float consist = 0.5;
            if(max_consist_ != min_consist_)
                consist = (CandidateTrajectories_[i].consistency - min_consist_) / (max_consist_ - min_consist_);
            CandidateTrajectories_[i].consistency = consist;

            float score = wr_*safeness + ws_*smoothness + wc_*consist;
            CandidateTrajectories_[i].score = score;
        }
    }
}


void BCPlannerLocal::evaluateTrajectory(int index)
{
    CandidateTrajectories_[index].bObstacle = false;
    CandidateTrajectories_[index].bFeasible = true;

    float fTInterval = 0.05;
    float ft = 0.0;

    float cum_cur = 0.0;
    int num_cur = 1;

    float cum_obs = 0.0;
    int num_obs = 1;

    int cellData = 0;
    float curvature = 0.0;

    while(ft <= 1.0)
    {
        if(ft>=1.0)
        {
            ft = 1.0;
            evaluatePoint(index, ft, cellData, curvature);
            ft = 1.2;
        }
        else
        {
            evaluatePoint(index, ft, cellData, curvature);
        }

        if(ft == 0.0)
        {
            CandidateTrajectories_[index].c0 = curvature;
        }


        if(cellData >= 99)
        {
            CandidateTrajectories_[index].bObstacle = true;     
            return;
        }
        else if(cellData >= 0)
        {

            cum_obs += cellData;
            num_obs++;
        }


        if(fabs(curvature) > 0.25)
        {
            CandidateTrajectories_[index].bFeasible = false;
            return;
        }
        else
        {
            cum_cur += fabs(curvature);
            num_cur++;
        }
        ft = ft + fTInterval;
    }

    CandidateTrajectories_[index].smoothness = cum_cur/num_cur;
    CandidateTrajectories_[index].safeness = cum_obs/num_obs;
    CandidateTrajectories_[index].consistency = fabs(CandidateTrajectories_[index].c0 - lastCurvature_);

    if(max_smooth_ < CandidateTrajectories_[index].smoothness)
        max_smooth_ = CandidateTrajectories_[index].smoothness;
    if(min_smooth_ > CandidateTrajectories_[index].smoothness)
        min_smooth_ = CandidateTrajectories_[index].smoothness;

    if(max_safe_ < CandidateTrajectories_[index].safeness)
        max_safe_ = CandidateTrajectories_[index].safeness;
    if(min_safe_ > CandidateTrajectories_[index].safeness)
        min_safe_ = CandidateTrajectories_[index].safeness;

    if(max_consist_ < CandidateTrajectories_[index].consistency)
        max_consist_ = CandidateTrajectories_[index].consistency;
    if(min_consist_ > CandidateTrajectories_[index].consistency)
        min_consist_ = CandidateTrajectories_[index].consistency;

}

void BCPlannerLocal::evaluatePoint(int index, float ft, int& cellData, float &fCurvature)
{
    geometry_msgs::Point P0 = CandidateTrajectories_[index].P0;
    geometry_msgs::Point P1 = CandidateTrajectories_[index].P1;
    geometry_msgs::Point P2 = CandidateTrajectories_[index].P2;
    geometry_msgs::Point P3 = CandidateTrajectories_[index].P3;

    /** get point */
    float fPX = (1-ft)*(1-ft)*(1-ft)*P0.x+3*(1-ft)*(1-ft)*ft*P1.x+3*(1-ft)*ft*ft*P2.x+ft*ft*ft*P3.x;
    float fPY = (1-ft)*(1-ft)*(1-ft)*P0.y+3*(1-ft)*(1-ft)*ft*P1.y+3*(1-ft)*ft*ft*P2.y+ft*ft*ft*P3.y;

    /** get point derivatives*/
    float fDiffX = 3*(1-ft)*(1-ft)*(P1.x-P0.x)+6*(1-ft)*ft*(P2.x-P1.x)+3*ft*ft*(P3.x-P2.x);
    float fDiffY = 3*(1-ft)*(1-ft)*(P1.y-P0.y)+6*(1-ft)*ft*(P2.y-P1.y)+3*ft*ft*(P3.y-P2.y);

    /** get point dderivatives*/
    float fDDiffX = 6*(1-ft)*(P2.x-2*P1.x+P0.x)+6*ft*(P3.x-2*P2.x+P1.x);
    float fDDiffY = 6*(1-ft)*(P2.y-2*P1.y+P0.y)+6*ft*(P3.y-2*P2.y+P1.y);

    /** get occupancy cell data */

    if (!occupymap_.getMapData(fPX, fPY, cellData))
    {
        cellData = -1;
    }
    /**  get curvature*/
    fCurvature = (fDiffX*fDDiffY-fDiffY*fDDiffX)/pow(fDiffX*fDiffX+fDiffY*fDiffY ,1.5);
}


Data BCPlannerLocal::getTrainData(float optim_c0)
{
    vector<float> feasibles;
    vector<float> trables;
    vector<float> consists;

    int reindex = 0;

    int optimal_index = -1;
    float min_c0_dist = 100.0;

    int i = 0;
    cout<<optim_c0<<"|";
    for(i=0; i<nCandidateNum_; i++)
    {
        if(CandidateTrajectories_[i].bFeasible==false || CandidateTrajectories_[i].bObstacle==true)
            continue;

        feasibles.push_back(CandidateTrajectories_[i].smoothness);
        trables.push_back(CandidateTrajectories_[i].safeness);
        consists.push_back(CandidateTrajectories_[i].consistency);

        float c0_dist = fabs(CandidateTrajectories_[i].c0 - optim_c0);
        cout<<c0_dist<<"|";
        if(c0_dist < min_c0_dist)
        {
            min_c0_dist = c0_dist;
            optimal_index = reindex;
        }
        reindex++;

    }
    cout<<endl;
    cout<<"optimal_index:"<<optimal_index<<"; min c0 dist:"<<min_c0_dist<<endl;

    Data data;
    data.d1s = trables;
    data.d2s = feasibles;
    data.d3s = consists;
    data.candidate_num = reindex;
    data.optimal_index = optimal_index;

    return data;
}

nav_msgs::Path BCPlannerLocal::getCandidatePathToRVIZ()
{
    nav_msgs::Path CandidatePathToRVIZ;
    for(int i=0; i<nCandidateNum_; i++)
    {
        geometry_msgs::Point P0 = CandidateTrajectories_[i].P0;
        geometry_msgs::Point P1 = CandidateTrajectories_[i].P1;
        geometry_msgs::Point P2 = CandidateTrajectories_[i].P2;
        geometry_msgs::Point P3 = CandidateTrajectories_[i].P3;


        float ft = 0.0;
        for(int j=0; j<20; j++)
        {
            ft = j*0.05;
            geometry_msgs::PoseStamped pose;
            /** get point */
            pose.pose.position.x = (1-ft)*(1-ft)*(1-ft)*P0.x+3*(1-ft)*(1-ft)*ft*P1.x+3*(1-ft)*ft*ft*P2.x+ft*ft*ft*P3.x;
            pose.pose.position.y = (1-ft)*(1-ft)*(1-ft)*P0.y+3*(1-ft)*(1-ft)*ft*P1.y+3*(1-ft)*ft*ft*P2.y+ft*ft*ft*P3.y;
            CandidatePathToRVIZ.poses.push_back(pose);
        }

        for(int j=20; j>=0; j--)
        {
            ft = j*0.05;
            geometry_msgs::PoseStamped pose;
            /** get point */
            pose.pose.position.x = (1-ft)*(1-ft)*(1-ft)*P0.x+3*(1-ft)*(1-ft)*ft*P1.x+3*(1-ft)*ft*ft*P2.x+ft*ft*ft*P3.x;
            pose.pose.position.y = (1-ft)*(1-ft)*(1-ft)*P0.y+3*(1-ft)*(1-ft)*ft*P1.y+3*(1-ft)*ft*ft*P2.y+ft*ft*ft*P3.y;
            CandidatePathToRVIZ.poses.push_back(pose);
        }
    }
    return CandidatePathToRVIZ;
}

nav_msgs::Path BCPlannerLocal::getFeasiblePathToRVIZ()
{
    nav_msgs::Path FeasiblePathToRVIZ;
    for(int i=0; i<nCandidateNum_; i++)
    {
        if(CandidateTrajectories_[i].bFeasible == false || CandidateTrajectories_[i].bObstacle == true)
            continue;

        geometry_msgs::Point P0 = CandidateTrajectories_[i].P0;
        geometry_msgs::Point P1 = CandidateTrajectories_[i].P1;
        geometry_msgs::Point P2 = CandidateTrajectories_[i].P2;
        geometry_msgs::Point P3 = CandidateTrajectories_[i].P3;


        float ft = 0.0;
        for(int j=0; j<20; j++)
        {
            ft = j*0.05;
            geometry_msgs::PoseStamped pose;
            /** get point */
            pose.pose.position.x = (1-ft)*(1-ft)*(1-ft)*P0.x+3*(1-ft)*(1-ft)*ft*P1.x+3*(1-ft)*ft*ft*P2.x+ft*ft*ft*P3.x;
            pose.pose.position.y = (1-ft)*(1-ft)*(1-ft)*P0.y+3*(1-ft)*(1-ft)*ft*P1.y+3*(1-ft)*ft*ft*P2.y+ft*ft*ft*P3.y;
            FeasiblePathToRVIZ.poses.push_back(pose);
        }

        for(int j=20; j>=0; j--)
        {
            ft = j*0.05;
            geometry_msgs::PoseStamped pose;
            /** get point */
            pose.pose.position.x = (1-ft)*(1-ft)*(1-ft)*P0.x+3*(1-ft)*(1-ft)*ft*P1.x+3*(1-ft)*ft*ft*P2.x+ft*ft*ft*P3.x;
            pose.pose.position.y = (1-ft)*(1-ft)*(1-ft)*P0.y+3*(1-ft)*(1-ft)*ft*P1.y+3*(1-ft)*ft*ft*P2.y+ft*ft*ft*P3.y;
            FeasiblePathToRVIZ.poses.push_back(pose);
        }
    }
    return FeasiblePathToRVIZ;
}

nav_msgs::Path BCPlannerLocal::getOptimalPathToRVIZ()
{
    nav_msgs::Path OptimalPathToRVIZ;
    geometry_msgs::Point P0 = CandidateTrajectories_[nOptimalIndex_].P0;
    geometry_msgs::Point P1 = CandidateTrajectories_[nOptimalIndex_].P1;
    geometry_msgs::Point P2 = CandidateTrajectories_[nOptimalIndex_].P2;
    geometry_msgs::Point P3 = CandidateTrajectories_[nOptimalIndex_].P3;


    float ft = 0.0;
    for(int j=0; j<20; j++)
    {
        ft = j*0.05;
        geometry_msgs::PoseStamped pose;
        /** get point */
        pose.pose.position.x = (1-ft)*(1-ft)*(1-ft)*P0.x+3*(1-ft)*(1-ft)*ft*P1.x+3*(1-ft)*ft*ft*P2.x+ft*ft*ft*P3.x;
        pose.pose.position.y = (1-ft)*(1-ft)*(1-ft)*P0.y+3*(1-ft)*(1-ft)*ft*P1.y+3*(1-ft)*ft*ft*P2.y+ft*ft*ft*P3.y;
        OptimalPathToRVIZ.poses.push_back(pose);
    }

    return OptimalPathToRVIZ;
}




}

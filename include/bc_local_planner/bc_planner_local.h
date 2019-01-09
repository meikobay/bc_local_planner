#ifndef BC_LOCAL_PLANNER_BC_PLANNER_LOCAL_H_
#define BC_LOCAL_PLANNER_BC_PLANNER_LOCAL_H_
#include "ros/ros.h"
#include "bc_local_planner/occupy_map.h"
#include "bc_local_planner/MCLearner.h"
#include "string"
#include "iostream"
#include "vector"

#include "nav_msgs/Path.h"
#include "geometry_msgs/Point.h"

using std::cout;
using std::endl;
using std::vector;



namespace bc_local_planner {

struct TargetPoint {
    float fX, fY, fHeading;
};

struct CubicBezierTrajectory {
    geometry_msgs::Point P0, P1, P2, P3;
    float smoothness, safeness, consistency, score;
    bool bObstacle, bFeasible;
    float c0;
};

//struct Data
//{
//    vector<float> smoothness;
//    vector<float> safeness;
//    vector<float> consists;
//    int optimal_index;
//    int candidate_num;
//};

class BCPlannerLocal {
public:
    BCPlannerLocal();
    ~BCPlannerLocal();

    void InitialData();

    void updateWeigths(float wr, float ws, float wc)
    {
        wr_ = wr;
        ws_ = ws;
        wc_ = wc;
    }

    void updateOccupyMap(OccupyMap occupymap)
    {
        occupymap_ = occupymap;
    }

    bool getCurrentCurvature(float& curvature);

    void genearteCandidate(geometry_msgs::Point ugv_pose, geometry_msgs::Point target_pose);  // map point.z to yaw

    void evaluateTrajectory(int index);

    void evaluatePoint(int index, float ft, int &cellData, float& fCurvature);

    Data getTrainData(float optim_c0);


    nav_msgs::Path getCandidatePathToRVIZ();
    nav_msgs::Path getFeasiblePathToRVIZ();
    nav_msgs::Path getOptimalPathToRVIZ();


private:
    float wr_, ws_, wc_;
    OccupyMap occupymap_;

    int nDistSampleNum_;
    float fDistSampleInterval_;
    int nSampleNum_;
    int nCandidateNum_;

    int nOptimalIndex_;

    float* DistP0P1_;
    float* DistP2P3_;

    float lastCurvature_;

    float max_smooth_;
    float min_smooth_;

    float max_safe_;
    float min_safe_;

    float max_consist_;
    float min_consist_;

    TargetPoint* TargetSamples_;
    CubicBezierTrajectory* CandidateTrajectories_;

};

}
#endif

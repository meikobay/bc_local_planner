#ifndef BC_LOCAL_PLANNER_BC_PLANNER_GLOBAL_H_
#define BC_LOCAL_PLANNER_BC_PLANNER_GLOBAL_H_
#include "ros/ros.h"
#include "string"
#include "iostream"
#include "bc_local_planner/occupy_map.h"
#include "nav_msgs/Path.h"
#include "tf/LinearMath/Matrix3x3.h"
#include "tf/tf.h"
#include "tf2/LinearMath/Matrix3x3.h"


using std::cout;
using std::endl;


namespace bc_local_planner {

struct BiBezierRCL{
    geometry_msgs::Point P0, P1, P2;
    float smoothness, safeness, target, score;
};


class BCPlannerGlobal {
public:
    BCPlannerGlobal();
    ~BCPlannerGlobal();

    void InitialData();

    void updateWeigths(float wr, float ws, float wt)
    {
        wr_ = wr;
        ws_ = ws;
        wt_ = wt;
    }

    void updateOccupyMap(OccupyMap occupymap)
    {
        occupymap_ = occupymap;
    }


    geometry_msgs::Point getCurrentLG();

    void genearteCandidate(geometry_msgs::Point ugv_pose, geometry_msgs::Point cwp_point);  // map point.z to yaw

    void evaluateTrajectory(int index, geometry_msgs::Point cwp_point);

    void evaluatePoint(int index, float ft, int &cellData);

    nav_msgs::Path getCandidateRCLToRVIZ();
    nav_msgs::Path getOptimalRCLToRVIZ();

private:
    float wr_, ws_, wt_;
    OccupyMap occupymap_;

    int nAngleSampleNum_;
    float fAngleSampleInterval_;
    int nSampleNum_;
    float fLookHeadDist_;
    float fP1Dist_;
    float fLGDist_;

    float max_smooth_;
    float min_smooth_;

    float max_safe_;
    float min_safe_;

    float max_target_;
    float min_target_;

    nav_msgs::Path OptimalRCL_;

    BiBezierRCL* CandidateRCLs_;
};

}
#endif

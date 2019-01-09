#include "bc_local_planner/occupy_map.h"


OccupyMap::OccupyMap() : initialized_(false){}

OccupyMap::~OccupyMap(){}

void OccupyMap::updateMap(const nav_msgs::OccupancyGrid &map)
{
    if(!initialized_) initialized_ = true;

    size_x_ = map.info.width;
    size_y_ = map.info.height;
    origin_x_ = map.info.origin.position.x;
    origin_y_ = map.info.origin.position.y;
    resolution_ = map.info.resolution;
    map_ = map;
}


void OccupyMap::mapToWorld(unsigned int mx, unsigned int my, float &wx, float &wy) const
{
    wx = origin_x_ + (mx + 0.5) * resolution_;
    wy = origin_y_ + (my + 0.5) * resolution_;
}

bool OccupyMap::worldToMap(float wx, float wy, unsigned int &mx, unsigned int &my) const
{
    if (wx < origin_x_ || wy < origin_y_)
        return false;

    mx = (int)((wx - origin_x_) / resolution_);
    my = (int)((wy - origin_y_) / resolution_);

    if (mx < size_x_ && my < size_y_)
        return true;

    return false;
}

bool OccupyMap::getMapData(float wx, float wy, int &occupy_data) const
{
    unsigned int mx, my;
    if(!worldToMap(wx, wy, mx, my))
        return false;

    unsigned int index = getIndex(mx, my);
    occupy_data = map_.data[index];
    return true;
}

#ifndef OCCUPY_MAP_H_
#define OCCUPY_MAP_H_
#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "std_msgs/Int8MultiArray.h"


class OccupyMap {
public:
    /**
     * @brief  Constructor for OccupyMap wrapper
     */
    OccupyMap();
    /**
     * @brief  Destructor for the wrapper
     */
    ~OccupyMap();
    void updateMap(const nav_msgs::OccupancyGrid& map);
    /**
     * @brief  Convert from world coordinates to map coordinates
     * @param  wx The x world coordinate
     * @param  wy The y world coordinate
     * @param  mx Will be set to the associated map x coordinate
     * @param  my Will be set to the associated map y coordinate
     * @return True if the conversion was successful (legal bounds) false otherwise
     */
    bool worldToMap(float wx, float wy, unsigned int& mx, unsigned int& my) const;
    /**
     * @brief  Convert from map coordinates to world coordinates
     * @param  mx The x map coordinate
     * @param  my The y map coordinate
     * @param  wx Will be set to the associated world x coordinate
     * @param  wy Will be set to the associated world y coordinate
     */
    void mapToWorld(unsigned int mx, unsigned int my, float& wx, float& wy) const;
    /**
     * @brief  Given two map coordinates... compute the associated index
     * @param mx The x coordinate
     * @param my The y coordinate
     * @return The associated index
     */
    inline unsigned int getIndex(unsigned int mx, unsigned int my) const
    {
        return my * size_x_ + mx;
    }
    /**
     * @brief  Given an index... compute the associated map coordinates
     * @param  index The index
     * @param  mx Will be set to the x coordinate
     * @param  my Will be set to the y coordinate
     */
    inline void indexToCells(unsigned int index, unsigned int& mx, unsigned int& my) const
    {
        my = index / size_x_;
        mx = index - (my * size_x_);
    }

    bool getMapData(float wx, float wy, int &occupy_data) const;

private:
    bool initialized_;
    unsigned int size_x_;
    unsigned int size_y_;
    float resolution_;
    float origin_x_;
    float origin_y_;
    nav_msgs::OccupancyGrid map_;
};

#endif

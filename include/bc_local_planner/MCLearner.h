#ifndef MCLEARNER_H_
#define MCLEARNER_H_
#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "vector"
#include <random>
#include <boost/random.hpp>
#include <boost/random/uniform_int.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/cauchy_distribution.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/multivariate_normal_distribution.hpp>


using std::vector;
using std::cout;
using std::endl;

struct Data
{
    vector<float> d1s;
    vector<float> d2s;
    vector<float> d3s;
    int optimal_index;
    int candidate_num;
};

class MCLearner {
public:
    /**
     * @brief  Constructor for MCLearner wrapper
     */
    MCLearner();
    /**
     * @brief  Destructor for the wrapper
     */
    ~MCLearner();

    void setWeights(float w1, float w2, float w3);
    void getWeights(float &w1, float &w2, float &w3);
    void feedLearner(vector<Data> data_set);
    vector<geometry_msgs::Point> generateSamples(int sampleNum, float w1, float w2, float w3);
    void elevateWeight(vector<Data> data_set);



private:
    float w1_, w2_, w3_;
    int maxIter_;
    float resampleRate_;
};

#endif

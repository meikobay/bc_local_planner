#include "bc_local_planner/MCLearner.h"

MCLearner::MCLearner():
    w1_(0.3),
    w2_(0.3),
    w3_(0.4),
    maxIter_(100),
    resampleRate_(0.1)
{

}

MCLearner::~MCLearner(){}


void MCLearner::setWeights(float w1, float w2, float w3)
{
    w1_ = w1;
    w2_ = w2;
    w3_ = w3;
}

void MCLearner::getWeights(float &w1, float &w2, float &w3)
{
    w1 = w1_;
    w2 = w2_;
    w3 = w3_;
}


void MCLearner::feedLearner(vector<Data> data_set)
{

    float w1, w2, w3;
    getWeights(w1, w2, w3);
    int sample_num = 1000;
    for(int iter_num = 0; iter_num<maxIter_; iter_num++)
    {
        vector<geometry_msgs::Point> samples = generateSamples(sample_num, w1, w2, w3);


        vector<int> scores;
        for(int i=0; i<sample_num; i++)
        {
            int score = 1;
            for(int j=0; j<data_set.size(); j++)
            {
                Data data = data_set[j];
                int oi = data.optimal_index;
                float optimal_cost = samples[i].x*data.d1s[oi]
                        +samples[i].y*data.d2s[oi]
                        +samples[i].z*data.d3s[oi];

                for(int k=0; k<data.candidate_num; k++)
                {
                    float cost = samples[i].x*data.d1s[k]
                                +samples[i].y*data.d2s[k]
                                +samples[i].z*data.d3s[k];
                    if(optimal_cost>cost)
                        score++;

                }

            }
//            cout<<i<<":"<<"score:"<<score<<";";
            scores.push_back(score);
        }

        vector<size_t> idx(scores.size());
        iota(idx.begin(), idx.end(), 0);
        sort(idx.begin(), idx.end(), [&scores](size_t i1, size_t i2){return scores[i1]<scores[i2];});
        sort(scores.begin(), scores.end());


        float wr_sum = 0.0;
        float ws_sum = 0.0;
        float wc_sum = 0.0;
        int resample_num = (int)(sample_num*resampleRate_);


        for(int i=0; i<resample_num; i++)
        {
            int index = idx[i];
            wr_sum = wr_sum+samples[index].x;
            ws_sum = ws_sum+samples[index].y;
            wc_sum = wc_sum+samples[index].z;
        }

        w1 = wr_sum/resample_num;
        w2 = ws_sum/resample_num;
        w3 = wc_sum/resample_num;

        float wrsc = w1+w2+w3;


        w1 = w1/wrsc;
        w2 = w2/wrsc;
        w3 = w3/wrsc;

        if(fabs(w1-w1_)<0.01 && fabs(w2-w2_)<0.01 && fabs(w3-w3_)<0.01)
        {
            w1_ = w1;
            w2_ = w2;
            w3_ = w3;
            break;
        }
        else
        {
            w1_ = w1;
            w2_ = w2;
            w3_ = w3;
        }

    }

    cout<<"new weight:"<<w1_<<";"<<w2_<<";"<<w3_<<endl;

    elevateWeight(data_set);
}

void MCLearner::elevateWeight(vector<Data> data_set)
{
    vector<int> scores;
    cout<<"elevate result:";

    for(int j=0; j<data_set.size(); j++)
    {
        int score = 1;
        Data data = data_set[j];
        int oi = data.optimal_index;
        float optimal_cost = w1_*data.d1s[oi]
                +w2_*data.d2s[oi]
                +w3_*data.d3s[oi];

        for(int k=0; k<data.candidate_num; k++)
        {
            float cost = w1_*data.d1s[k]
                        +w2_*data.d2s[k]
                        +w3_*data.d3s[k];
            if(optimal_cost>cost)
                score++;

        }
        cout<<score<<",";

        scores.push_back(score);
    }
    cout<<endl;



}

vector<geometry_msgs::Point> MCLearner::generateSamples(int sampleNum, float w1, float w2, float w3)
{
    boost::numeric::ublas::matrix<double> std_value(3,3);

    std_value(0,0) = 0.1;
    std_value(0,1) = 0.0;
    std_value(0,2) = 0.0;
    std_value(1,0) = 0.0;
    std_value(1,1) = 0.1;
    std_value(1,2) = 0.0;
    std_value(2,0) = 0.0;
    std_value(2,1) = 0.0;
    std_value(2,2) = 0.1;

    boost::numeric::ublas::vector<double> mean_value(3);

    mean_value(0) = w1;
    mean_value(1) = w2;
    mean_value(2) = w3;



    boost::mt19937 generator(time(0)*rand());
    boost::multivariate_normal_distribution<> multivariate_generator(std_value, mean_value);
    boost::variate_generator<boost::mt19937&, boost::multivariate_normal_distribution<> > multivariate_numbers(generator, multivariate_generator);
    vector<geometry_msgs::Point> samples;
    int sample_size = 1;

    while(1)
    {
        geometry_msgs::Point point;
        point.x = multivariate_numbers();
        point.y = multivariate_numbers();
        point.z = multivariate_numbers();

        if(point.x < 0.0 || point.x > 1.0 || point.y < 0.0 || point.y > 1.0 || point.z < 0.0 || point.z > 1.0)
            continue;

        float sum = point.x+point.y+point.z;
        point.x = point.x/sum;
        point.y = point.y/sum;
        point.z = point.z/sum;

        samples.push_back(point);
//            cout<<sample_size<<","<<point<<endl;
        sample_size++;
        if(sample_size > sampleNum)
            break;
    }
    return samples;
}

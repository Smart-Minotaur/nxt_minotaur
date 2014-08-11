#ifndef MINOTAUR_MAP_MOVEMENT_HPP_
#define MINOTAUR_MAP_MOVEMENT_HPP_

#include <geometry_msgs/TwistWithCovariance.h>

namespace minotaur
{
    /**
     * \brief Is a structure for storing the robot's linear and angular
     *        velocity and its covariance.
     */
    class Movement
    {
    public:
        float v;
        float w;
        float  sigma[2][2];
        
        Movement() { }
        Movement(const float p_v, const float p_w)
        :v(p_v), w(p_w) { }
        virtual ~Movement() { }
        
        void set(const float p_v, const float p_w)
        { v = p_v; w = p_w; }
        
        void setSigma(float **p_sigma)
        {
            for(int i = 0; i < 2; ++i)
                for(int j = 0;  j < 2; ++j)
                    sigma[i][j] = p_sigma[i][j];
        }
        
        void convert(const geometry_msgs::TwistWithCovariance& p_twist)
        {
            int i, j;
            double theta = atan2(p_twist.twist.linear.y, p_twist.twist.linear.x);
            v = cos(theta) / p_twist.twist.linear.x;
            for(i = 0; i < 2; ++i)
                for(j = 0; j < 2; ++j)
                    sigma[j][i] = p_twist.covariance[j * 6 + i];
        }
        
        
    };
}

#endif

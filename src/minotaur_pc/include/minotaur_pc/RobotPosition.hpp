#ifndef MINOTAUR_PC_ROBOT_POSITION_HPP_
#define MINOTAUR_PC_ROBOT_POSITION_HPP_

#include <tf/transform_broadcaster.h>
#include "minotaur_pc/Vec2.hpp"
#include "geometry_msgs/PoseWithCovariance.h"

namespace minotaur
{
    class RobotPosition
    {
    public:
        Vec2 point;
        float theta;
        float sigma[3][3];
        
        RobotPosition() { }
        virtual ~RobotPosition() { }
        
        void setSigma(float **p_sigma)
        {
            for(int i = 0; i < 3; ++i)
                for(int j = 0;  j < 3; ++j)
                    sigma[i][j] = p_sigma[i][j];
        }
        
        void convert(const geometry_msgs::PoseWithCovariance& p_pose)
        {
            int i, j;
            point.x = p_pose.pose.position.x;
            point.y = p_pose.pose.position.y;
            theta = tf::getYaw(p_pose.pose.orientation);
            for(i = 0; i < 3; ++i)
                for(j = 0; j < 3; ++j)
                    sigma[j][i] = p_pose.covariance[j * 6 + i];
        }
    };
}

#endif
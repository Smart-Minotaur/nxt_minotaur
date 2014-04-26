#ifndef MINOTAUR_PC_ROBOT_POSITION_HPP_
#define MINOTAUR_PC_ROBOT_POSITION_HPP_

#include "minotaur_pc/Vec2.hpp"

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
    };
}

#endif
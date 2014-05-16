#include <math.h>

#include "minotaur_pc/BugZeroAlgorithm.hpp"

namespace minotaur
{
 
    RobotPosition BugZeroAlgorithm::getNextPosition(const RobotPosition p_pos)
    {
        float x = p_pos.point.x;
        float y = p_pos.point.y;
        float theta = p_pos.theta;
        bool obstacle = true;
        
        while(obstacle)
        {
            obstacle = checkObstacle(x, y, theta);
            if(obstacle)
            {
                theta += 90;
                if(theta == 360)
                    theta = 0;
            }
        }
        
        int delta_x = (int) (sin(theta) * MIN_DISTANCE_TO_OBSTACLE);
        int delta_y = (int) (cos(theta) * MIN_DISTANCE_TO_OBSTACLE);
        
        RobotPosition newPosition;
        newPosition.point.x = x + delta_x;
        newPosition.point.y = y + delta_y;
        newPosition.theta = theta;
        //newPosition.sigma = p_pos.sigma;

        return newPosition;
    }
    
}
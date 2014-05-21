#include <cmath>

#include "minotaur_pc/BugZeroAlgorithm.hpp"

namespace minotaur
{
 
    RobotPosition BugZeroAlgorithm::getNextPosition(const RobotPosition p_pos)
    {
        int x = (int) METER_TO_CENTIMETER(p_pos.point.x);
        int y = (int) METER_TO_CENTIMETER(p_pos.point.y);
        float theta = p_pos.theta;
        bool obstacle = true;
        int noWayCounter = 0;
        
        while(obstacle)
        {
            obstacle = checkObstacle(x, y, theta);
            if(obstacle)
            {
                theta += (float)(M_PI/2);
                noWayCounter++;
                if(noWayCounter == 4)
                {
                    return p_pos;
                }
            }
        }

        

        RobotPosition newPosition;
        
        //newPosition.sigma = p_pos.sigma;
        newPosition = calculateNewPosition(x, y, theta);
        return newPosition;
    }
    
}
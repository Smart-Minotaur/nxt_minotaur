#include <cmath>
#include "minotaur_map/BugZeroAlgorithm.hpp"
#include "minotaur_common/Math.hpp"

namespace minotaur
{
 
    RobotPosition BugZeroAlgorithm::getNextPosition(const RobotPosition p_pos)
    {
        
        float x = meterToCm(p_pos.point.x);
        float y = meterToCm(p_pos.point.y);
        float theta = p_pos.theta;
        bool frontObstacle = false;
        
        // mutex lock
        
        RobotPosition newPosition;
        
        frontObstacle = checkFrontObstacle();
        
        if(!frontObstacle)
        {
            newPosition = calculateNewPosition(x, y, theta);
        }
        else
        {
            newPosition.point.x = p_pos.point.x;
            newPosition.point.y = p_pos.point.y;
            newPosition.theta = theta + M_PI/2;
        }
        
        // mutex unlock
        return newPosition;
    }
    
}

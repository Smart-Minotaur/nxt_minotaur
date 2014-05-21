
#include <cmath>

#include "minotaur_pc/PledgeAlgorithm.hpp"

namespace minotaur
{
    PledgeAlgorithm::PledgeAlgorithm(Map* pMap) :MovementAlgorithm(pMap)
    {
        turn = 0;
        obstacleFound = false;
    }
    
    
    RobotPosition PledgeAlgorithm::getNextPosition(const RobotPosition p_pos)
    {
        int x = (int) METER_TO_CENTIMETER(p_pos.point.x);
        int y = (int) METER_TO_CENTIMETER(p_pos.point.y);
        float theta = p_pos.theta;
        bool obstacle = false;
        bool leftSideObstacle = false;
        RobotPosition newPosition;
        
        
        if(turn == 0)
        {
            if(!obstacle)
            {
                newPosition = calculateNewPosition(x, y, theta);
                return newPosition;
            }
            else
            {
                obstacleFound = true;
                while(obstacle) // evtl sackgasse
                {
                    theta -= M_PI/2;
                    turn--;
                    
                    obstacle = checkObstacle(x, y, theta);
                }
                newPosition = calculateNewPosition(x, y, theta);
                return newPosition;
            }
        }
        
        if(turn != 0)
        {
            leftSideObstacle = checkLeftSideObstacle(x, y, theta);
            
            if(leftSideObstacle)
            {
                newPosition = calculateNewPosition(x, y, theta);
                return newPosition;
            }
            else
            {
                theta += M_PI/2; //drehung nach links?
                turn++;
                newPosition = calculateNewPosition(x, y, theta);
                return newPosition;
            }
        }
    }
 
 
    
    
    
 
}
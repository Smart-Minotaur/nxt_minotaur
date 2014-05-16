
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
        int x = p_pos.point.x;
        int y = p_pos.point.y;
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
                    theta -= 90; //drehung nach rechts?
                    if(theta == -360)
                    {
                        theta = 0;
                    }
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
                theta += 90; //drehung nach links?
                if(theta == 360)
                {
                    theta = 0;
                }
                turn++;
                newPosition = calculateNewPosition(x, y, theta);
                return newPosition;
            }
            
        }
    }
 
 
    
    
    
 
}
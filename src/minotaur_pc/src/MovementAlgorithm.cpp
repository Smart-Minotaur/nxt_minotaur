
#include <math.h>
#include "minotaur_pc/MovementAlgorithm.hpp"




namespace minotaur
{
    MovementAlgorithm::MovementAlgorithm(Map* pMap)
    {
        map = pMap;
    }
    
    
    
    bool MovementAlgorithm::checkObstacle(int x, int y, float theta)
    {
        int** field = map->getField();
        for(int i = 1; i < MIN_DISTANCE_TO_OBSTACLE; ++i)
        {
            int delta_x = (int) (sin(theta) * i);
            int delta_y = (int) (cos(theta) * i);
            
            if(field[x + delta_x][y + delta_y] > OBSTACLE_THRESHOLD) // found obstacle
            {
                return true;
            }
        }
        return false;
    }
    
    bool MovementAlgorithm::checkLeftSideObstacle(int x, int y, float theta)
    {
        int** field = map->getField();
        /*for(int i = 1; i < MIN_DISTANCE_TO_OBSTACLE; ++i)
        {
            int delta_x = (int) (sin(theta) * i);
            int delta_y = (int) (cos(theta) * i);
            
            if(field[x + delta_x][y + delta_y] > OBSTACLE_THRESHOLD) // found obstacle
            {
                return true;
            }
        }*/
        return false;
    }
    
    bool MovementAlgorithm::checkRightSideObstacle(int x, int y, float theta)
    {
        int** field = map->getField();
        /*for(int i = 1; i < MIN_DISTANCE_TO_OBSTACLE; ++i)
        {
            int delta_x = (int) (sin(theta) * i);
            int delta_y = (int) (cos(theta) * i);
            
            if(field[x + delta_x][y + delta_y] > OBSTACLE_THRESHOLD) // found obstacle
            {
                return true;
            }
        }*/
        return false;
    }

    
    RobotPosition MovementAlgorithm::calculateNewPosition(int x, int y, int theta)
    {
        RobotPosition pos;
        
        int delta_x = (int) (sin(theta) * MIN_DISTANCE_TO_OBSTACLE);
        int delta_y = (int) (cos(theta) * MIN_DISTANCE_TO_OBSTACLE);
        pos.point.x = x + delta_x;
        pos.point.y = y + delta_y;
        pos.theta = theta;
        
        return pos;
    }

}
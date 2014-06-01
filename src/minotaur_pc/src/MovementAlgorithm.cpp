#include <cmath>
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
        theta += M_PI/2;
        return checkObstacle(x, y, theta);
    }
    
    bool MovementAlgorithm::checkRightSideObstacle(int x, int y, float theta)
    {
        theta -= M_PI/2; 
        return checkObstacle(x, y, theta);
    }

    
    RobotPosition MovementAlgorithm::calculateNewPosition(int x, int y, int theta)
    {
        RobotPosition pos;
        
        int leftDistance = getDistanceToLeftObstacle(x, y, theta);
        int rightDistance = getDistanceToRightObstacle(x, y, theta);
        
        if(leftDistance < MIN_DISTANCE_TO_OBSTACLE)
        {
            int diff = MIN_DISTANCE_TO_OBSTACLE - leftDistance;
            if(diff != 0)
            {
                y -= diff;
            }
        }
        if(rightDistance < MIN_DISTANCE_TO_OBSTACLE)
        {
            int diff = MIN_DISTANCE_TO_OBSTACLE - rightDistance;
            if(diff != 0)
            {
                y += diff;
            }
        }
        
        int delta_x = (int) (sin(theta) * MIN_DISTANCE_TO_OBSTACLE);
        int delta_y = (int) (cos(theta) * MIN_DISTANCE_TO_OBSTACLE);
        pos.point.x = CENTIMETER_TO_METER(x + delta_x);
        pos.point.y = CENTIMETER_TO_METER(y + delta_y);
        
        
        while(theta < 0)
        {
            theta += (float)(2 * M_PI);
        }
        while(theta >= (float)2 * M_PI)
        {
            theta -= (float)(2 * M_PI);
        }
        
        pos.theta = theta;
        
        return pos;
    }
    
    int MovementAlgorithm::getDistanceToObstacle(int x, int y, float theta)
    {
        int** field = map->getField();
        for(int i = 1; i < MIN_DISTANCE_TO_OBSTACLE; ++i)
        {
            int delta_x = (int) (sin(theta) * i);
            int delta_y = (int) (cos(theta) * i);
            
            if(field[x + delta_x][y + delta_y] > OBSTACLE_THRESHOLD) // found obstacle
            {
                
                return i;
            }
        }
        return 0;
    }
    
    int MovementAlgorithm::getDistanceToLeftObstacle(int x, int y, float theta)
    {
        theta += M_PI/2;        
        return getDistanceToObstacle(x, y, theta);
    }
    
    
    
    int MovementAlgorithm::getDistanceToRightObstacle(int x, int y, float theta)
    {
        theta -= M_PI/2;
        return getDistanceToObstacle(x, y, theta);
    }
    
    
    

}
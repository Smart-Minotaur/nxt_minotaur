#ifndef MINOTAUR_MAP_MOVEMENT_ALGORITHM_HEADER_HPP_
#define MINOTAUR_MAP_MOVEMENT_ALGORITHM_HEADER_HPP_

#include "minotaur_map/Map.hpp"
#include "minotaur_map/RobotPosition.hpp"

#define MIN_DISTANCE_TO_OBSTACLE 10
#define OBSTACLE_THRESHOLD 2

namespace minotaur
{
    class MovementAlgorithm
    {
    private:
        Map* map;

    public:
        MovementAlgorithm(Map* pMap);
        virtual ~MovementAlgorithm(){}
        
        virtual RobotPosition getNextPosition(const RobotPosition pos) = 0;
        bool checkObstacle(int x, int y, float theta);
        bool checkLeftSideObstacle(int x, int y, float theta);
        bool checkRightSideObstacle(int x, int y, float theta);
        
        int getDistanceToObstacle(int x, int y, float theta);
        int getDistanceToLeftObstacle(int x, int y, float theta);
        int getDistanceToRightObstacle(int x, int y, float theta);
        
        
        RobotPosition calculateNewPosition(int x, int y, int theta);
      
    };
    
}


#endif

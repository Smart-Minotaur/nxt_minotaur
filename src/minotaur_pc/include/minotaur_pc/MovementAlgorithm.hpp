#ifndef MINOTAUR_PC_MOVEMENT_ALGORITHM_HEADER_HPP_
#define MINOTAUR_PC_MOVEMENT_ALGORITHM_HEADER_HPP_

#include "minotaur_pc/Map.hpp"
#include "minotaur_pc/RobotPosition.hpp"

#define MIN_DISTANCE_TO_OBSTACLE 10
#define OBSTACLE_THRESHOLD 2

#define METER_TO_CENTIMETER(m) m*100
#define CENTIMETER_TO_METER(cm) cm/100

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
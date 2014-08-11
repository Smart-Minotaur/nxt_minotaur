#ifndef MINOTAUR_MAP_MOVEMENT_ALGORITHM_HEADER_HPP_
#define MINOTAUR_MAP_MOVEMENT_ALGORITHM_HEADER_HPP_

#include "minotaur_map/Map.hpp"
#include "minotaur_map/RobotPosition.hpp"

#define MIN_DISTANCE_TO_OBSTACLE 10
#define OBSTACLE_THRESHOLD 2

namespace minotaur
{
    /**
     * \brief An abstract class for implmenting algorithms to explore a map.
     * 
     * This class is an abstract class for implementing algorithms to
     * explore a map.
     * The exploration algorithm works depending on a already finished
     * Map object.
     */
    class MovementAlgorithm
    {
    private:
        Map* map;

    public:
        MovementAlgorithm(Map* pMap);
        virtual ~MovementAlgorithm(){}
        
        /**
         * Calculates the next target position, to which the robot should
         * move.
         * @param pos current position of the robot
         * @return next target position of the robot
         */
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

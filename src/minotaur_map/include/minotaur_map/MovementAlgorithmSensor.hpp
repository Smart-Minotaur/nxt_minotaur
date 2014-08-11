#ifndef MINOTAUR_MAP_MOVEMENT_ALGORITHM_SENSOR_HEADER_HPP_
#define MINOTAUR_MAP_MOVEMENT_ALGORITHM_SENSOR_HEADER_HPP_

#include "minotaur_map/Map.hpp"
#include "minotaur_map/RobotPosition.hpp"

#define MIN_DISTANCE_TO_OBSTACLE 8
#define MAX_DISTANCE 30

#define FRONT_SENSOR 1
#define LEFT_SENSOR 3
#define RIGHT_SENSOR 2


namespace minotaur
{
    class MovementAlgorithmSensor
    {
    private:
      
        int m_distanceFrontSensor;
        int m_distanceLeftSensor;
        int m_distanceRightSensor;

    public:
        MovementAlgorithmSensor() {};
        virtual ~MovementAlgorithmSensor(){};
        
        virtual RobotPosition getNextPosition(const RobotPosition pos) = 0;
        void setSensorValue(const int p_sensor, const int p_distance);
        
        bool checkFrontObstacle();
        RobotPosition calculateNewPosition(float x, float y, float theta);
    };
}

#endif

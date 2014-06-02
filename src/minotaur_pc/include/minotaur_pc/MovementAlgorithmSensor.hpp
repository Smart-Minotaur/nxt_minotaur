#ifndef MINOTAUR_PC_MOVEMENT_ALGORITHM_SENSOR_HEADER_HPP_
#define MINOTAUR_PC_MOVEMENT_ALGORITHM_SENSOR_HEADER_HPP_


#include "minotaur_pc/Map.hpp"
#include "minotaur_pc/RobotPosition.hpp"


#define MIN_DISTANCE_TO_OBSTACLE 8
#define MAX_DISTANCE 30

#define METER_TO_CENTIMETER(m) m*100
#define CENTIMETER_TO_METER(cm) cm/100

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
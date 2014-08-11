#include "minotaur_map/MovementAlgorithmSensor.hpp"
#include "minotaur_common/Math.hpp"

namespace minotaur
{

    void MovementAlgorithmSensor::setSensorValue(const int p_sensor, const int p_distance)
    {
        switch(p_sensor)
        {
            case FRONT_SENSOR:
                m_distanceFrontSensor = p_distance;
                break;
            
            case LEFT_SENSOR:
                m_distanceLeftSensor = p_distance;
                break;
            
            case RIGHT_SENSOR:
                m_distanceRightSensor = p_distance;
                break;
        }
    }

    bool MovementAlgorithmSensor::checkFrontObstacle()
    {
        if(m_distanceFrontSensor < MIN_DISTANCE_TO_OBSTACLE)
        {
            return true;
        }
        return false;
    }
    
    RobotPosition MovementAlgorithmSensor::calculateNewPosition(float x, float y, float theta)
    {
        float delta_x;
        float delta_y;
        
        RobotPosition newPosition;
        
        if(m_distanceFrontSensor > MAX_DISTANCE)
        {
            delta_x = sin(theta) * MAX_DISTANCE;
            delta_y = cos(theta) * MAX_DISTANCE;
        }
        else 
        {
            delta_x = sin(theta) * (m_distanceFrontSensor - MIN_DISTANCE_TO_OBSTACLE);
            delta_y = cos(theta) * (m_distanceFrontSensor - MIN_DISTANCE_TO_OBSTACLE);
        }
        
        newPosition.point.x = cmToMeter(x + delta_x);
        newPosition.point.y = cmToMeter(y + delta_y);
        newPosition.theta = theta;
        
        
        return newPosition; 
    }
    

}

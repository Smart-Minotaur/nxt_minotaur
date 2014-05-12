#include <ros/ros.h>
#include "minotaur_pc/MinotaurState.hpp"
#include "nxt_beagle/Config.hpp"

namespace minotaur
{
    MinotaurState::MinotaurState()
    {
        pthread_mutex_init(&positionMutex, NULL);
        pthread_mutex_init(&movementMutex, NULL);
        pthread_mutex_init(&measureMutex, NULL);
    }
    
    MinotaurState::~MinotaurState()
    {
        pthread_mutex_destroy(&positionMutex);
        pthread_mutex_destroy(&movementMutex);
        pthread_mutex_destroy(&measureMutex);
    }
    
    SensorMeasurement MinotaurState::getMeasurement(const int p_sensorId)
    {
        pthread_mutex_lock(&measureMutex);
        SensorMeasurement result = sensorMeasurements[p_sensorId];
        pthread_mutex_unlock(&measureMutex);
        
        return result;
    }
    
    void MinotaurState::setDistance(const int p_sensorId, const uint8_t p_distance)
    {
        pthread_mutex_lock(&measureMutex);
        sensorMeasurements[p_sensorId].distance = p_distance;
        pthread_mutex_unlock(&measureMutex);
    }
    
    void MinotaurState::setMeasurement(const int p_sensorId, const SensorMeasurement &p_measurement)
    {
        pthread_mutex_lock(&measureMutex);
        sensorMeasurements[p_sensorId] = p_measurement;
        pthread_mutex_unlock(&measureMutex);
    }
    
    int MinotaurState::addMeasurement(const SensorMeasurement &p_measurement)
    {
        pthread_mutex_lock(&measureMutex);
        sensorMeasurements.push_back(p_measurement);
        int result = sensorMeasurements.size() - 1;
        pthread_mutex_unlock(&measureMutex);
        
        return result;
    }
    
    void MinotaurState::clearMeasurements()
    {
        pthread_mutex_lock(&measureMutex);
        sensorMeasurements.clear();
        pthread_mutex_unlock(&measureMutex);
    }
    
    void MinotaurState::setVelocity(const float p_linearVel, const float p_angularVel)
    {
        pthread_mutex_lock(&movementMutex);
        lastMeasuredVel.linearVelocity = p_linearVel;
        lastMeasuredVel.angularVelocity = p_angularVel;
        pthread_mutex_unlock(&movementMutex);
    }
    
    void MinotaurState::setMovement(const Movement &p_movement)
    {
        pthread_mutex_lock(&movementMutex);
        lastMeasuredVel = p_movement;
        pthread_mutex_unlock(&movementMutex);
    }
    
    Movement MinotaurState::getMovement()
    {
        pthread_mutex_lock(&movementMutex);
        Movement result = lastMeasuredVel;
        pthread_mutex_unlock(&movementMutex);
        
        return result;
    }
    
    void MinotaurState::setPosition(const RobotPosition &p_position)
    {
        pthread_mutex_lock(&positionMutex);
        position = p_position;
        pthread_mutex_unlock(&positionMutex);
    }
    
    void MinotaurState::setPosition(const Vec2 &p_position)
    {
        pthread_mutex_lock(&positionMutex);
        position.point = p_position;
        pthread_mutex_unlock(&positionMutex);
    }
    
    RobotPosition MinotaurState::getPosition()
    {
        pthread_mutex_lock(&positionMutex);
        RobotPosition result = position;
        pthread_mutex_unlock(&positionMutex);
        
        return result;
    }
    
    void MinotaurState::setOdometry(const RobotPosition &p_position, const Movement &p_movement)
    {
        pthread_mutex_lock(&positionMutex);
        pthread_mutex_lock(&movementMutex);
        position = p_position;
        lastMeasuredVel = p_movement;
        pthread_mutex_unlock(&movementMutex);
        pthread_mutex_unlock(&positionMutex);
    }
    
    void MinotaurState::setModel(const std::string &p_model)
    {
        std::exception tmpException;
        bool except = false;
        SensorMeasurement tmpMeasurment;
        
        pthread_mutex_lock(&measureMutex);
        try
        {
            sensorMeasurements.clear();
            int id = 0;
            while(true)
            {
                if(!ros::param::has(PARAM_SENSOR(p_model, id)))
                    break;
                
                ros::param::get(PARAM_SENSOR_DX(p_model, id), tmpMeasurment.offset.x);
                ros::param::get(PARAM_SENSOR_DY(p_model, id), tmpMeasurment.offset.y);
                ros::param::get(PARAM_SENSOR_DIRECTION(p_model, id), tmpMeasurment.direction);
                
                sensorMeasurements.push_back(tmpMeasurment);
                ++id;
            }
        }
        catch(std::exception const &e)
        {
            except = true;
            tmpException = e;
        }
        
        pthread_mutex_unlock(&measureMutex);
        
        if(except)
            throw tmpException;
    }
}
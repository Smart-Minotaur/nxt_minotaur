#ifndef MINOTAUR_PC_MINOTAUR_STATE_HPP_
#define MINOTAUR_PC_MINOTAUR_STATE_HPP_

#include <pthread.h>
#include <vector>
#include <string>
#include "minotaur_pc/Movement.hpp"
#include "minotaur_pc/SensorMeasurement.hpp"
#include "minotaur_pc/RobotPosition.hpp"
#include "minotaur_pc/Vec2.hpp"

namespace minotaur
{
    class MinotaurState
    {
    private:
        RobotPosition position;
        Movement lastMeasuredVel;
        std::vector<SensorMeasurement> sensorMeasurements;
        
        pthread_mutex_t positionMutex;
        pthread_mutex_t movementMutex;
        pthread_mutex_t measureMutex;
    public:
        MinotaurState();
        virtual ~MinotaurState();
        
        SensorMeasurement getMeasurement(const int p_sensorId);
        void setDistance(const int p_sensorId, const uint8_t p_distance);
        void setMeasurement(const int p_sensorId, const SensorMeasurement &p_measurement);
        int addMeasurement(const SensorMeasurement &p_measurement);
        void clearMeasurements();
        
        void setVelocity(const float p_linearVel, const float p_angularVel);
        void setMovement(const Movement &p_movement);
        Movement getMovement();
        
        void setPosition(const RobotPosition &p_position);
        void setPosition(const Vec2 &p_position);
        RobotPosition getPosition();
        
        void setOdometry(const RobotPosition &p_position, const Movement &p_movement);
        
        void setModel(const std::string &p_model);
    };
}

#endif
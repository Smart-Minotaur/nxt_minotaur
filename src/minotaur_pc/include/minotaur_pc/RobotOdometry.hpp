#ifndef MINOTAUR_PC_ROBOT_ODOMETRY_HPP_
#define MINOTAUR_PC_ROBOT_ODOMETRY_HPP_

#include <pthread.h>
#include <vector>
#include <string>
#include "minotaur_pc/Movement.hpp"
#include "minotaur_pc/SensorMeasurement.hpp"
#include "minotaur_pc/RobotPosition.hpp"
#include "minotaur_pc/Vec2.hpp"

namespace minotaur
{
    class RobotOdometry
    {
    private:
        RobotPosition position;
        Movement lastMeasuredVel;
        
        pthread_mutex_t positionMutex;
        pthread_mutex_t movementMutex;
    public:
        RobotOdometry();
        virtual ~RobotOdometry();
        
        void setVelocity(const float p_linearVel, const float p_angularVel);
        void setMovement(const Movement &p_movement);
        Movement getMovement();
        
        void setPosition(const RobotPosition &p_position);
        void setPosition(const Vec2 &p_position);
        RobotPosition getPosition();
        
        void setOdometry(const RobotPosition &p_position, const Movement &p_movement);
    };
}

#endif
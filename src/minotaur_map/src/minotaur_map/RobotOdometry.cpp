#include <ros/ros.h>
#include "minotaur_map/RobotOdometry.hpp"
#include "minotaur_common/RAIILock.hpp"

namespace minotaur
{
    RobotOdometry::RobotOdometry()
    {
        pthread_mutex_init(&positionMutex, NULL);
        pthread_mutex_init(&movementMutex, NULL);
    }
    
    RobotOdometry::~RobotOdometry()
    {
        pthread_mutex_destroy(&positionMutex);
        pthread_mutex_destroy(&movementMutex);
    }
    
    void RobotOdometry::setVelocity(const float p_linearVel, const float p_angularVel)
    {
        RAIILock lock(&movementMutex);
        lastMeasuredVel.v = p_linearVel;
        lastMeasuredVel.w = p_angularVel;
    }
    
    void RobotOdometry::setMovement(const Movement &p_movement)
    {
        RAIILock lock(&movementMutex);
        lastMeasuredVel = p_movement;
    }
    
    Movement RobotOdometry::getMovement()
    {
        RAIILock lock(&movementMutex);
        return lastMeasuredVel;
    }
    
    void RobotOdometry::setPosition(const RobotPosition &p_position)
    {
        RAIILock lock(&positionMutex);
        position = p_position;
    }
    
    void RobotOdometry::setPosition(const Vec2 &p_position)
    {
        RAIILock lock(&positionMutex);
        position.point = p_position;
    }
    
    RobotPosition RobotOdometry::getPosition()
    {
        RAIILock lock(&positionMutex);
        return position;
    }
    
    void RobotOdometry::setOdometry(const RobotPosition &p_position, const Movement &p_movement)
    {
        RAIILock lock1(&positionMutex);
        RAIILock lock2(&movementMutex);
        position = p_position;
        lastMeasuredVel = p_movement;
    }
}

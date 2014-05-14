#include <ros/ros.h>
#include "minotaur_pc/RobotOdometry.hpp"
#include "nxt_beagle/Config.hpp"

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
        pthread_mutex_lock(&movementMutex);
        lastMeasuredVel.v = p_linearVel;
        lastMeasuredVel.w = p_angularVel;
        pthread_mutex_unlock(&movementMutex);
    }
    
    void RobotOdometry::setMovement(const Movement &p_movement)
    {
        pthread_mutex_lock(&movementMutex);
        lastMeasuredVel = p_movement;
        pthread_mutex_unlock(&movementMutex);
    }
    
    Movement RobotOdometry::getMovement()
    {
        pthread_mutex_lock(&movementMutex);
        Movement result = lastMeasuredVel;
        pthread_mutex_unlock(&movementMutex);
        
        return result;
    }
    
    void RobotOdometry::setPosition(const RobotPosition &p_position)
    {
        pthread_mutex_lock(&positionMutex);
        position = p_position;
        pthread_mutex_unlock(&positionMutex);
    }
    
    void RobotOdometry::setPosition(const Vec2 &p_position)
    {
        pthread_mutex_lock(&positionMutex);
        position.point = p_position;
        pthread_mutex_unlock(&positionMutex);
    }
    
    RobotPosition RobotOdometry::getPosition()
    {
        pthread_mutex_lock(&positionMutex);
        RobotPosition result = position;
        pthread_mutex_unlock(&positionMutex);
        
        return result;
    }
    
    void RobotOdometry::setOdometry(const RobotPosition &p_position, const Movement &p_movement)
    {
        pthread_mutex_lock(&positionMutex);
        pthread_mutex_lock(&movementMutex);
        position = p_position;
        lastMeasuredVel = p_movement;
        pthread_mutex_unlock(&movementMutex);
        pthread_mutex_unlock(&positionMutex);
    }
}
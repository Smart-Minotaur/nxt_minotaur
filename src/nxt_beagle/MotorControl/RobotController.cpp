/*
 * Author: Fabian Meyer 
 */

#include "nxt_beagle/RobotController.hpp"
#include "nxt_beagle/MotorVelocity.hpp"

namespace minotaur
{
    const RobotVelocity& RobotController::getRobotVelocity() const
    {
        return velocity;
    }
    
    IPIDController& RobotController::getPIDController()
    {
        return pidController;
    }
    
    float RobotController::getWheelTrack() const
    {
        return wheelTrack;
    }
    
    RobotVelocity RobotController::getMeasuredVelocity() const
    {
        RobotVelocity result;
        MotorVelocity motorVel = pidController.getMeasuredVelocity();
        
        //to get the formula see kinematic of two wheeled robots
        result.linearVelocity = (motorVel.rightMPS + motorVel.leftMPS) / 2;
        result.angularVelocity = (motorVel.rightMPS - motorVel.leftMPS) / wheelTrack;
        return result;
    }

    void RobotController::setRobotVelocity(const RobotVelocity& p_velocity)
    {
        velocity = p_velocity;
        calculateMotorVelocity();
    }
    
    void RobotController::setWheelTrack(const float p_wheelTrack)
    {
        wheelTrack = p_wheelTrack;
        calculateMotorVelocity();
    }

    void RobotController::calculateMotorVelocity()
    {
        MotorVelocity targetVelocity;
        
        //to get the formula see kinematic of two wheeled robots
        targetVelocity.leftMPS = velocity.linearVelocity - (velocity.angularVelocity * wheelTrack) / 2;
        targetVelocity.rightMPS = velocity.linearVelocity + (velocity.angularVelocity * wheelTrack) / 2;
        
        pidController.setVelocity(targetVelocity);
    }
    
    void RobotController::step(const int p_samplingIntervallMsec)
    {
        pidController.step(p_samplingIntervallMsec);
    }
}
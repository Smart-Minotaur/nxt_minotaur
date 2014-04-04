#ifndef IROBOT_CONTROLLER_HPP_
#define IROBOT_CONTROLLER_HPP_

#include "nxt_beagle/IPIDController.hpp"

namespace minotaur
{
    class RobotVelocity
    {
    public:
        float linearVelocity;
        float angularVelocity;
        
        RobotVelocity()
        :linearVelocity(0.0f), angularVelocity(0.0f) { }
        
        RobotVelocity(const float p_linearVel, const float p_angularVel)
        :linearVelocity(p_linearVel), angularVelocity(p_angularVel) { }
        
        virtual ~RobotVelocity() { }
        
        void set(const float p_linearVel, const float p_angularVel)
        { linearVelocity = p_linearVel; angularVelocity = p_angularVel;}
    };
        
    class IRobotController
    {
    public:
        IRobotController() { }
        virtual ~IRobotController() { }
        
        virtual const RobotVelocity& getRobotVelocity() const = 0;
        virtual IPIDController& getPIDController() = 0;
        virtual float getWheelTrack() const = 0;
        
        virtual void setRobotVelocity(const RobotVelocity& p_velocity) = 0;
        virtual void setWheelTrack(const float p_wheelTrack) = 0;
        
        virtual void step(const float p_samplingIntervall) = 0;
    };
}

#endif
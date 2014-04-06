#ifndef IROBOT_CONTROLLER_HPP_
#define IROBOT_CONTROLLER_HPP_

#include "nxt_beagle/IPIDController.hpp"
#include "nxt_beagle/RobotVelocity.hpp"

namespace minotaur
{
    class IRobotController
    {
    public:
        IRobotController() { }
        virtual ~IRobotController() { }
        
        virtual const RobotVelocity& getRobotVelocity() const = 0;
        virtual IPIDController& getPIDController() = 0;
        virtual float getWheelTrack() const = 0;
        virtual RobotVelocity getMeasuredVelocity() const = 0;
        
        virtual void setRobotVelocity(const RobotVelocity& p_velocity) = 0;
        virtual void setWheelTrack(const float p_wheelTrack) = 0;
        
        virtual void step(const float p_samplingIntervall) = 0;
    };
}

#endif
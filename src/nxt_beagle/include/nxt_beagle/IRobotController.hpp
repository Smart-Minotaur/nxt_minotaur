/*
 * Author: Fabian Meyer 
 */

#ifndef IROBOT_CONTROLLER_HPP_
#define IROBOT_CONTROLLER_HPP_

#include "nxt_beagle/IPIDController.hpp"
#include "nxt_beagle/RobotVelocity.hpp"

namespace minotaur
{
    /* This class provides an Interface to control the Robot.
     * The robot is controlled by setting the linear- and angular velocity.
     * To keep the right velocity, a PID controller is used (see IPIDController.hpp). */
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
        
        virtual void step(const int p_samplingIntervallMSec) = 0;
    };
}

#endif
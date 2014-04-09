/*
 * Author: Fabian Meyer 
 */

#ifndef ROBOT_CONTROLLER_HPP_
#define ROBOT_CONTROLLER_HPP_

#include "nxt_beagle/IRobotController.hpp"
#include "nxt_beagle/PIDController.hpp"

namespace minotaur
{
    /* This class implments an Interface to control a robot.
     * The class receives linear and angular velocity and calculates the
     * velocity for each motor (left and right).
     * A pid-controller is used to keep the correct velocity on each motor.
     * The pid-controller has to be initialized before using the "step()"
     * method (see "PIDController.hpp").
     * The wheel-track must be ste before using the "step()" method. */
    class RobotController: public IRobotController
    {
    private:
        RobotVelocity velocity;
        float wheelTrack;
        PIDController pidController;
        
        void calculateMotorVelocity();
        
    public:
        RobotController()
        : velocity(), wheelTrack(0.0f), pidController() { }
        
        virtual ~RobotController() { }
        
        const RobotVelocity& getRobotVelocity() const;
        IPIDController& getPIDController();
        float getWheelTrack() const;
        RobotVelocity getMeasuredVelocity() const;
        
        void setRobotVelocity(const RobotVelocity& p_velocity);
        void setWheelTrack(const float p_wheelTrack);
        
        void step(const int p_samplingIntervallMSec);
    };
}

#endif
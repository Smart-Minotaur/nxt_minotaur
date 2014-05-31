/*
 * Author: Fabian Meyer 
 */

#ifndef ROBOT_CONTROL_IPID_CONTROLLER_HPP
#define ROBOT_CONTROL_IPID_CONTROLLER_HPP

#include "robot_control/MotorVelocity.hpp"
#include "robot_control/PIDParameter.hpp"
#include "nxt_control/Motor.hpp"

namespace minotaur
{
    /* This class provides an Interface for a PID-controller.
     * Look up PID-controller for more information about PID-controller.
     * The controller is used to regulate the velocity of the robot. */
    class IPIDController
    {
        public:
            IPIDController() { }
            virtual ~IPIDController() { }
            
            virtual void setLeftMotor(nxtcon::Motor *p_leftMotor) = 0;
            virtual void setRightMotor(nxtcon::Motor *p_rightMotor) = 0;
            
            virtual void setVelocity(const MotorVelocity& p_velocity) = 0;
            virtual void setWheelCircumference(const float p_meter) = 0;
            virtual void setPIDParameter(const PIDParameter& p_param) = 0;
            
            virtual const MotorVelocity& getVelocity() const = 0;
            virtual const MotorVelocity& getMeasuredVelocity() const = 0;
            virtual float getWheelCircumference() const = 0;
            virtual const PIDParameter& getPIDParameter() const = 0;
            
            virtual void step(const int p_samplingIntervallMSec) = 0;
    };
}

#endif
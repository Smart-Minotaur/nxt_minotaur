#ifndef ROBOT_CONTROL_IPID_CONTROLLER_HPP
#define ROBOT_CONTROL_IPID_CONTROLLER_HPP

#include "minotaur_common/MotorVelocity.hpp"
#include "minotaur_common/PIDParameter.h"
#include "nxt_control/Motor.hpp"

namespace minotaur
{
    /** 
     * \brief This class provides an Interface for a PID-controller.
     * 
     * Interface for the PIDController that is used to regulate 
     * the velocity of the robot.
     */
    class IPIDController
    {
        public:
            IPIDController() { }
            virtual ~IPIDController() { }
            
            virtual void setLeftMotor(nxtcon::Motor *p_leftMotor) = 0;
            virtual void setRightMotor(nxtcon::Motor *p_rightMotor) = 0;
            
            virtual void setVelocity(const MotorVelocity& p_velocity) = 0;
            virtual void setWheelRadius(const float p_meter) = 0;
            virtual void setPIDParameter(const minotaur_common::PIDParameter& p_param) = 0;
            
            virtual const MotorVelocity& getVelocity() const = 0;
            virtual const MotorVelocity& getMeasuredVelocity() const = 0;
            virtual float getWheelRadius() const = 0;
            virtual const minotaur_common::PIDParameter& getPIDParameter() const = 0;
            
            virtual void step(const int p_samplingIntervalMSec) = 0;
    };
}

#endif

/*
 * Author: Fabian Meyer 
 */

#ifndef IPID_CONTROLLER_HPP
#define IPID_CONTROLLER_HPP

#include <ros/ros.h>
#include "nxt_beagle/MotorVelocity.hpp"
#include "nxt_beagle/PIDParameter.hpp"

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
            
            virtual void setMotorPublisher(ros::Publisher *p_motorPublisher ) = 0;
            virtual void setMotorClient(ros::ServiceClient *p_motorClient) = 0;
            
            virtual void setVelocity(const MotorVelocity& p_velocity) = 0;
            virtual void setWheelCircumference(const float p_meter) = 0;
            virtual void setPIDParameter(const PIDParameter& p_param) = 0;
            
            virtual const MotorVelocity& getVelocity() const = 0;
            virtual const MotorVelocity& getMeasuredVelocity() const = 0;
            virtual float getWheelCircumference() const = 0;
            virtual const PIDParameter& getPIDParameter() const = 0;
            
            virtual void step(const float p_samplingIntervallSecs) = 0;
    };
}

#endif
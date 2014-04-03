#ifndef IPID_CONTROLLER_HPP
#define IPID_CONTROLLER_HPP

#include <ros/ros.h>
#include "nxt_beagle/MotorVelocity.hpp"

namespace minotaur
{ 
    class PIDParameter
    {
    public:
        float Kp;
        float Ki;
        float Kd;
        
        PIDParameter()
        : Kp(0), Ki(0), Kd(0) { }
        
        PIDParameter(const float p_Kp, const float p_Ki, const float p_Kd)
        : Kp(p_Kp), Ki(p_Ki), Kd(p_Kd) { }
        
        ~PIDParameter() { }
        
        void set(const float p_Kp, const float p_Ki, const float p_Kd)
        { Kp = p_Kp; Ki = p_Ki; Kd = p_Kd; }
    };
    
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
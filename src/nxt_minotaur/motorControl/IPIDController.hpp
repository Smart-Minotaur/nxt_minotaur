#ifndef IPID_CONTROLLER_HPP
#define IPID_CONTROLLER_HPP

#include <ros/ros.h>

namespace minotaur
{ 
    class IPIDController
    {
        public:
            IPIDController() { }
            virtual ~IPIDController() { }
            
            virtual void setMotorPublisher(ros::Publisher *p_motorPublisher ) = 0;
            virtual void setMotorClient(ros::ServiceClient *p_motorClient) = 0;
            
            virtual void setLeftMPS(const float p_mps) = 0;
            virtual void setRightMPS(const float p_mps) = 0;
            virtual void setMeasureDuration(const unsigned int p_milli) = 0;
            virtual void setWheelCircumference(const float p_meter) = 0;
            
            virtual float getLeftMPS() const = 0;
            virtual float getRightMPS() const = 0;
            virtual unsigned int getMeasureDurationMilli() const = 0;
            virtual float getWheelCircumference() const = 0;
            
            virtual void step(const float p_samplingIntervallSecs) = 0;
    };
}

#endif
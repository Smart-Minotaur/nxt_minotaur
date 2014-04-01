#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

#include "IPIDController.hpp"
#include "MotorVelocity.hpp"

namespace minotaur
{
    
    class PIDController: public IPIDController
    {
    private:
        ros::Publisher *motorPublisher;
        ros::ServiceClient *motorClient;
        
        MotorVelocity targetVelocity;
        MotorVelocity measuredVelocity;
        MotorVelocity currentDiff;
        MotorVelocity lastDiff;
        MotorVelocity diffSum;
        
        int powerLeft;
        int powerRight;
        
        unsigned int measureMS;
        float circumference;
        
        void measureCurrentVelocity(const float p_samplingIntervallSecs);
        MotorVelocity measureTickVelocity(const float p_samplingIntervallSecs);
        MotorVelocity measureMouseVelocity(const float p_samplingIntervallSecs);
        float ticksToMPS(unsigned long p_ticks);
        void calculateDifference();
        void setMotorPower(const float p_samplingIntervallSecs);
        int pidMotorPower(const float p_currentDiff,
                                 const float p_lastDiff,
                                 const float p_diffSum,
                                 const int p_motorPercent,
                                 const float p_samplingIntervallSecs);
        
    public:
        PIDController();
        virtual ~PIDController();
        
        void setMotorPublisher(ros::Publisher *p_motorPublisher );
        void setMotorClient(ros::ServiceClient *p_motorClient);
        void setWheelCircumference(const float p_meter);
        
        void setLeftMPS(const float p_mps);
        void setRightMPS(const float p_mps);
        float getWheelCircumference() const;
        
        float getLeftMPS() const;
        float getRightMPS() const;
        
        void step(const float p_samplingIntervallSecs);
    };

}

#endif
/*
 * Author: Fabian Meyer 
 */

#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

#include "nxt_beagle/IPIDController.hpp"

#define PID_CONTROLLER_DEBUG_NAME "PIDController_Debug"

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
        
        PIDParameter pidParameter;
        
        int powerLeft;
        int powerRight;
        
        float circumference;
        
        void measureCurrentVelocity(const float p_samplingIntervallSecs);
        MotorVelocity measureTickVelocity(const float p_samplingIntervallSecs);
        MotorVelocity measureMouseVelocity(const float p_samplingIntervallSecs);
        float ticksToMPS(const float p_ticksPS);
        void calculateDifference();
        void setMotorPower(const float p_samplingIntervallSecs);
        int pidMotorPower(const float p_currentDiff,
                                 const float p_lastDiff,
                                 const float p_diffSum,
                                 const int p_motorPercent,
                                 const float p_samplingIntervallSecs);
       void printDebugInfoPerStep(); 
       
        
    public:
        PIDController();
        virtual ~PIDController();
        
        void setMotorPublisher(ros::Publisher *p_motorPublisher );
        void setMotorClient(ros::ServiceClient *p_motorClient);
        
        void setVelocity(const MotorVelocity& p_velocity);
        void setWheelCircumference(const float p_meter);
        void setPIDParameter(const PIDParameter& p_param);
        
        const MotorVelocity& getVelocity() const;
        const MotorVelocity& getMeasuredVelocity() const;
        float getWheelCircumference() const;
        const PIDParameter& getPIDParameter() const;
        
        void step(const float p_samplingIntervallSecs);
    };

}

#endif
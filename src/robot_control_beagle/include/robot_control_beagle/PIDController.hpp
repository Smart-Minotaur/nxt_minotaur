/*
 * Author: Fabian Meyer 
 */

#ifndef ROBOT_CONTROL_PID_CONTROLLER_HPP
#define ROBOT_CONTROL_PID_CONTROLLER_HPP

#include "robot_control_beagle/IPIDController.hpp"

#define PID_CONTROLLER_DEBUG_NAME "PIDController_Debug"
#define MS_TO_SEC(ms) (((float) ms) / 1000.0f)

namespace minotaur
{
    /* This class implements an PID-Controller.
     * The motor-publisher is used to set the "power" of the robot motors.
     * The motor-client is used to read the current "tick count" of each motor.
     * Communication is done via the ROS-network. Publisher, Client and the
     * wheel-circumference have to be set manually before using the "step()" method.*/
    class PIDController: public IPIDController
    {
    private:
        nxtcon::Motor *leftMotor;
        nxtcon::Motor *rightMotor;
        
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
        
        void setLeftMotor(nxtcon::Motor *p_leftMotor);
        void setRightMotor(nxtcon::Motor *p_rightMotor);
        
        void setVelocity(const MotorVelocity& p_velocity);
        void setWheelCircumference(const float p_meter);
        void setPIDParameter(const PIDParameter& p_param);
        
        const MotorVelocity& getVelocity() const;
        const MotorVelocity& getMeasuredVelocity() const;
        float getWheelCircumference() const;
        const PIDParameter& getPIDParameter() const;
        
        void step(const int p_samplingIntervallMSec);
    };

}

#endif
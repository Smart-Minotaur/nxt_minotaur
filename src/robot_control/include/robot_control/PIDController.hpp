#ifndef ROBOT_CONTROL_PID_CONTROLLER_HPP
#define ROBOT_CONTROL_PID_CONTROLLER_HPP

#include "robot_control/IPIDController.hpp"

#define PID_CONTROLLER_DEBUG_NAME "PIDController_Debug"

namespace minotaur
{
    /**
     * \brief The PID-Controller regulates the velocity of the robot
     * 
     * The Controller gets measured values of the velocity, compares
     * them with the desired values, and then calculates new velocities.
     * The motor-publisher is used to set the "power" of the robot motors.
     * The motor-client is used to read the current "tick count" of each motor.
     * Publisher, Client and the wheel-circumference have to be set manually 
     * before using the "step()" method.
     */
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
        
        minotaur_common::PIDParameter pidParameter;
        
        int powerLeft;
        int powerRight;
        
        float wheelRadius;
        
        void measureCurrentVelocity(const float p_samplingIntervalSecs);
        MotorVelocity measureTickVelocity(const float p_samplingIntervalSecs);
        MotorVelocity measureMouseVelocity(const float p_samplingIntervalSecs);
        float ticksToMPS(const float p_ticksPS);
	
	/**
	 * Calculates the defference between the desired value and the measurements
	 */
        void calculateDifference();
	
	/**
	 * Sets the new motor-power for both motors.
	 */
        void setMotorPower(const float p_samplingIntervalSecs);
	
	/**
	 * Calculates the new power for a particular motor.
	 * @retval pidMotorPower caluculated power for the motor
	 */
        int pidMotorPower(const float p_currentDiff,
                                 const float p_lastDiff,
                                 const float p_diffSum,
                                 const int p_motorPercent,
                                 const float p_samplingIntervalSecs);
       void printDebugInfoPerStep(); 
       
        
    public:
        PIDController();
        virtual ~PIDController();
        
        void setLeftMotor(nxtcon::Motor *p_leftMotor);
        void setRightMotor(nxtcon::Motor *p_rightMotor);
        
        void setVelocity(const MotorVelocity& p_velocity);
        void setWheelRadius(const float p_meter);
        void setPIDParameter(const minotaur_common::PIDParameter& p_param);
        
        const MotorVelocity& getVelocity() const;
        const MotorVelocity& getMeasuredVelocity() const;
        float getWheelRadius() const;
        const minotaur_common::PIDParameter& getPIDParameter() const;
        
	/**
	 * Sets new motor-powers after calculate them
	 * @param p_samplingIntervalMSec duration of one samping-interval in Msec
	 */
        void step(const int p_samplingIntervalMSec);
    };

}

#endif

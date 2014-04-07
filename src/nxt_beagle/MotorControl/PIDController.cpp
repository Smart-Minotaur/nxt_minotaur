/*
 * Author: Fabian Meyer 
 */

#include "nxt_beagle/PIDController.hpp"
#include "nxt_beagle/nxtTicks.h"
#include "nxt_beagle/nxtPower.h"

#define MS_PER_SECOND 1000
#define DEGREE_PER_TICK 1.0f
#define DEGREE_PER_CIRCLE 360.0f
#define DEF_WHEEL_CIRCUMFERENCE 0.16f

#define DEF_KP 0.5f
#define DEF_KI 0.3f
#define DEF_KD 0.1f

#define MAX_MOTOR_POWER 90
#define MIN_MOTOR_POWER -90
#define MAX_DIFF_SUM_MPS 10
#define MIN_DIFF_SUM_MPS -10

namespace minotaur
{
    PIDController::PIDController()
    :motorPublisher(NULL), motorClient(NULL), targetVelocity(),measuredVelocity(),
    currentDiff(), lastDiff(), diffSum(), pidParameter(DEF_KP, DEF_KI, DEF_KD),
    circumference(DEF_WHEEL_CIRCUMFERENCE)
    { }

    PIDController::~PIDController()
    { }

    void PIDController::setMotorPublisher(ros::Publisher *p_motorPublisher )
    {
        motorPublisher = p_motorPublisher;
    }

    void PIDController::setMotorClient(ros::ServiceClient *p_motorClient)
    {
        motorClient = p_motorClient;
    }

    void PIDController::setVelocity(const MotorVelocity& p_velocity)
    {
        targetVelocity = p_velocity;
    }

    void PIDController::setWheelCircumference(const float p_meter)
    {
        circumference = p_meter;
    }
    
    void PIDController::setPIDParameter(const PIDParameter& p_param)
    {
        pidParameter = p_param;
    }

    const MotorVelocity& PIDController::getVelocity() const
    {
        return targetVelocity;
    }
    
    const MotorVelocity& PIDController::getMeasuredVelocity() const
    {
        return measuredVelocity;
    }

    float PIDController::getWheelCircumference() const
    {
        return circumference;
    }
    
     const PIDParameter& PIDController::getPIDParameter() const
     {
         return pidParameter;
     }

    void PIDController::step(const float p_samplingIntervallSecs)
    {
        measureCurrentVelocity(p_samplingIntervallSecs);
        
        calculateDifference();
        
        setMotorPower(p_samplingIntervallSecs);
        
        printDebugInfoPerStep();
    }

    void PIDController::measureCurrentVelocity(const float p_samplingIntervallSecs)
    {
        MotorVelocity tickVel, mouseVel;
        
        tickVel = measureTickVelocity(p_samplingIntervallSecs);
        //TODO mouse velocity must be measured, currently only tickVel is used 
        //mouseVel = measureMouseVelocity();
        mouseVel = tickVel;
        
        measuredVelocity = (tickVel + mouseVel) / 2;
    }

    MotorVelocity PIDController::measureTickVelocity(const float p_samplingIntervallSecs)
    {
        MotorVelocity result;
        ros::Time begin, end;
        nxt_beagle::nxtTicks tickSrv;
        float ticksPSLeft, ticksPSRight;
        
        //measure ticks, tick_count is automatically reset
        motorClient->call(tickSrv);
        
        //calculate ticks per second
        //samplingIntervall is needed, provided by the caller of step()
        //if the time is really correct must ensured by the caller
        ticksPSLeft = (float) (tickSrv.response.leftTicks) / p_samplingIntervallSecs;
        ticksPSRight = (float) (tickSrv.response.rightTicks) / p_samplingIntervallSecs;
        
        //convert ticks per second to meter per second
        result.set(ticksToMPS(ticksPSLeft), ticksToMPS(ticksPSRight));

        return result;
    }

    float PIDController::ticksToMPS(const float p_ticksPS)
    {
        float result;
        result = ((p_ticksPS * DEGREE_PER_TICK) / DEGREE_PER_CIRCLE) * circumference;
        return result;
    }

    MotorVelocity PIDController::measureMouseVelocity(const float p_samplingIntervallSecs)
    {
        MotorVelocity result;
        
        return result;
    }

    void PIDController::calculateDifference()
    {
        //calculate difference between 'should be' and 'currently is' velocity
        lastDiff = currentDiff;
        currentDiff = targetVelocity - measuredVelocity;
        diffSum += currentDiff;
        
        //diff sum has to be limited to prevent type overflow
        if(diffSum.leftMPS > MAX_DIFF_SUM_MPS)
            diffSum.leftMPS = MAX_DIFF_SUM_MPS;
        else if( diffSum.leftMPS < MIN_DIFF_SUM_MPS)
            diffSum.leftMPS = MIN_DIFF_SUM_MPS;
            
        if(diffSum.rightMPS > MAX_DIFF_SUM_MPS)
            diffSum.rightMPS = MAX_DIFF_SUM_MPS;
        else if( diffSum.rightMPS < MIN_DIFF_SUM_MPS)
            diffSum.rightMPS = MIN_DIFF_SUM_MPS;
    }

    void PIDController::setMotorPower(const float p_samplingIntervallSecs)
    {
        nxt_beagle::nxtPower msg;
        
        //if motor should stay still, no PID-Controller is needed
        //just put power to 0
        if(targetVelocity.leftMPS != 0)
            powerLeft = pidMotorPower(currentDiff.leftMPS,
                                    lastDiff.leftMPS,
                                    diffSum.leftMPS,
                                    powerLeft,
                                    p_samplingIntervallSecs);
        else
            powerLeft = 0;
        
        if(targetVelocity.rightMPS != 0)
            powerRight = pidMotorPower(currentDiff.rightMPS,
                                    lastDiff.rightMPS,
                                    diffSum.rightMPS,
                                    powerRight,
                                    p_samplingIntervallSecs);
        else
            powerRight = 0;
        
        msg.leftMotor = powerLeft;
        msg.rightMotor = powerRight;
        
        motorPublisher->publish(msg);
        ros::spinOnce();
    }

    int PIDController::pidMotorPower(const float p_currentDiff,
                                    const float p_lastDiff,
                                    const float p_diffSum,
                                    const int p_motorPercent,
                                    const float p_samplingIntervallSecs)
    {
        int tmp, result;
        float u_motor;
        
        //calculate power factor via PID-Controller
        u_motor = pidParameter.Kp * p_currentDiff + pidParameter.Ki * p_samplingIntervallSecs * p_diffSum + pidParameter.Kd * (p_currentDiff - p_lastDiff) / p_samplingIntervallSecs;
        tmp = u_motor * MAX_MOTOR_POWER;

        //limit motorpower
        result = p_motorPercent + tmp;
        if(result > MAX_MOTOR_POWER)
            result = MAX_MOTOR_POWER;
        if(result < MIN_MOTOR_POWER)
            result = MIN_MOTOR_POWER;
        
        return result;
    }
    
    void PIDController::printDebugInfoPerStep()
    {
        ROS_DEBUG_NAMED(PID_CONTROLLER_DEBUG_NAME, "=================================================");
        ROS_DEBUG_NAMED(PID_CONTROLLER_DEBUG_NAME, "Target Vel:     left = %.2f mps; right = %.2f mps", targetVelocity.leftMPS, targetVelocity.rightMPS);
        ROS_DEBUG_NAMED(PID_CONTROLLER_DEBUG_NAME, "Measured Vel:   left = %.2f mps; right = %.2f mps", measuredVelocity.leftMPS, measuredVelocity.rightMPS);
        ROS_DEBUG_NAMED(PID_CONTROLLER_DEBUG_NAME, "Cur Diff Vel:   left = %.2f mps; right = %.2f mps", currentDiff.leftMPS, currentDiff.rightMPS);
        ROS_DEBUG_NAMED(PID_CONTROLLER_DEBUG_NAME, "Last Diff Vel:  left = %.2f mps; right = %.2f mps", lastDiff.leftMPS, lastDiff.rightMPS);
        ROS_DEBUG_NAMED(PID_CONTROLLER_DEBUG_NAME, "Diff Sum:       left = %.2f mps; right = %.2f mps", diffSum.leftMPS, diffSum.rightMPS);
        ROS_DEBUG_NAMED(PID_CONTROLLER_DEBUG_NAME, "Power:          left = %d; right = %d", powerLeft, powerRight);
        ROS_DEBUG_NAMED(PID_CONTROLLER_DEBUG_NAME, "Circumference = %.4f m", circumference);
        ROS_DEBUG_NAMED(PID_CONTROLLER_DEBUG_NAME, "=================================================");
    }
}
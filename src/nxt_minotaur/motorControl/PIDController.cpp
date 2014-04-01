#include "PIDController.hpp"
#include "nxt_minotaur/nxtTicks.h"
#include "nxt_minotaur/nxtPower.h"

#define DEF_MEASURE_MS 15
#define MS_PER_SECOND 1000
#define DEGREE_PER_TICK 1
#define DEGREE_PER_CIRCLE 360
#define DEF_WHEEL_CIRCUMFERENCE 0.16f

#define KP 0.5f
#define KI 0.3f
#define KD 0.0f

#define MAX_MOTOR_POWER 90
#define MIN_MOTOR_POWER -90
#define MAX_DIFF_SUM_MPS 10
#define MIN_DIFF_SUM_MPS -10

using namespace minotaur;

PIDController::PIDController()
:motorPublisher(NULL), motorClient(NULL), targetVelocity(),measuredVelocity(),
currentDiff(), lastDiff(), diffSum(),
measureMS(DEF_MEASURE_MS), circumference(DEF_WHEEL_CIRCUMFERENCE)
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

void PIDController::setLeftMPS(const float p_mps)
{
    targetVelocity.leftMPS = p_mps;
}
void PIDController::setRightMPS(const float p_mps)
{
    targetVelocity.rightMPS = p_mps;
}

void PIDController::setMeasureDuration(const unsigned int p_milli)
{
    measureMS = p_milli;
}

void PIDController::setWheelCircumference(const float p_meter)
{
    circumference = p_meter;
}

float PIDController::getLeftMPS() const
{
    return targetVelocity.leftMPS;
}

float PIDController::getRightMPS() const
{
    return targetVelocity.rightMPS;
}

unsigned int PIDController::getMeasureDurationMilli() const
{
    return measureMS;
}

float PIDController::getWheelCircumference() const
{
    return circumference;
}

void PIDController::step(const float p_samplingIntervallSecs)
{
    measureCurrentVelocity();
    
    calculateDifference();
    
    setMotorPower(p_samplingIntervallSecs);
}

void PIDController::measureCurrentVelocity()
{
    MotorVelocity tickVel, mouseVel;
    
    tickVel = measureTickVelocity();
    //TODO mouse velocity must be measured, currently only tickVel is used 
    //mouseVel = measureMouseVelocity();
    mouseVel = tickVel;
    
    measuredVelocity = (tickVel + mouseVel) / 2;
    
    ROS_INFO("Measured Velocity: left = %.2fmps; right = %.2fmps", measuredVelocity.leftMPS, measuredVelocity.rightMPS);
}

MotorVelocity PIDController::measureTickVelocity()
{
    MotorVelocity result;
    ros::Time begin, end;
    nxt_minotaur::nxtTicks beginSrv, endSrv;
    unsigned long ticksPSLeft, ticksPSRight;
    
    //measure ticks for 'meaureDuration' milliseconds
    begin = ros::Time::now();
    
    motorClient->call(beginSrv);
    ros::Duration(measureMS / MS_PER_SECOND).sleep();
    motorClient->call(endSrv);
    
    end = ros::Time::now();
    
    //calculate ticks per second
    ticksPSLeft = (endSrv.response.leftTicks - beginSrv.response.leftTicks) / (end.toSec() - begin.toSec());
    ticksPSRight = (endSrv.response.rightTicks - beginSrv.response.rightTicks) / (end.toSec() - begin.toSec());
    
    //convert ticks per second to meter per second
    result.set(ticksToMPS(ticksPSLeft), ticksToMPS(ticksPSRight));

    return result;
}

float PIDController::ticksToMPS(unsigned long p_ticks)
{
    float result;
    result = ((p_ticks * DEGREE_PER_TICK) / DEGREE_PER_CIRCLE) * circumference;
    return result;
}

MotorVelocity PIDController::measureMouseVelocity()
{
    MotorVelocity result;
    
    return result;
}

void PIDController::calculateDifference()
{
    lastDiff = currentDiff;
    currentDiff = targetVelocity - measuredVelocity;
    diffSum += currentDiff;
    
    if(diffSum.leftMPS > MAX_DIFF_SUM_MPS)
        diffSum.leftMPS = MAX_DIFF_SUM_MPS;
    else if( diffSum.leftMPS < MIN_DIFF_SUM_MPS)
        diffSum.leftMPS = MIN_DIFF_SUM_MPS;
        
    if(diffSum.rightMPS > MAX_DIFF_SUM_MPS)
        diffSum.rightMPS = MAX_DIFF_SUM_MPS;
     else if( diffSum.rightMPS < MIN_DIFF_SUM_MPS)
        diffSum.rightMPS = MIN_DIFF_SUM_MPS;
    
    ROS_INFO("diffSum left = %.2f; right = %.2f", diffSum.leftMPS, diffSum.rightMPS);
    ROS_INFO("currentDiff left = %.2f; right = %.2f",currentDiff.leftMPS, currentDiff.rightMPS);
}

void PIDController::setMotorPower(const float p_samplingIntervallSecs)
{
    nxt_minotaur::nxtPower msg;
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

    u_motor = KP * p_currentDiff + KI * p_samplingIntervallSecs * p_diffSum + KD * (p_currentDiff - p_lastDiff) / p_samplingIntervallSecs;

    // attenuation of the sensibility of the sensor
    tmp = u_motor * MAX_MOTOR_POWER;

    // limitation of the minimum and maximum rotational speed
    result = p_motorPercent + tmp;
    if(result > MAX_MOTOR_POWER)
        result = MAX_MOTOR_POWER;
    if(result < MIN_MOTOR_POWER)
        result = MIN_MOTOR_POWER;
    
    ROS_INFO("Power = %d",result);
    ROS_INFO("U_MOTOR = %f", u_motor);
    
    return result;
}
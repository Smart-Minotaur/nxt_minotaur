#include <ros/ros.h>
#include "robot_control/PIDController.hpp"
#include "minotaur_common/Math.hpp"

#define DEG_PER_TICK 1.0f
#define DEF_WHEEL_RADIUS 0.1f
#define DEF_WHEEL_TRACK 0.1f

#define DEF_KP 0.5f
#define DEF_KI 0.3f
#define DEF_KD 0.1f

#define MAX_MOTOR_POWER 90
#define MIN_MOTOR_POWER -90
#define MAX_DIFF_SUM_MPS 10
#define MIN_DIFF_SUM_MPS -10

namespace minotaur
{
    PIDController::PIDController(nxt::Motor &p_leftMotor, nxt::Motor &p_rightMotor)
    :leftMotor(p_leftMotor), rightMotor(p_rightMotor), targetVelocity(), measuredVelocity(),
    currentDiff(), lastDiff(), diffSum(), pidParameter(),
    wheelRadius(DEF_WHEEL_RADIUS), wheelTrack(DEF_WHEEL_TRACK)
    {
        pidParameter.Kp = DEF_KP;
        pidParameter.Ki = DEF_KI;
        pidParameter.Kd = DEF_KD;
    }

    PIDController::~PIDController()
    { }
    
    void PIDController::setVelocity(const float p_linearVelocity, const float p_angularVelocity)
    {
		//to get the formula see kinematic of two wheeled robots
        targetVelocity.leftMPS = p_linearVelocity - (p_angularVelocity * wheelTrack) / 2;
		targetVelocity.rightMPS = p_linearVelocity + (p_angularVelocity * wheelTrack) / 2;
    }

    void PIDController::setWheelRadius(const float p_meter)
    {
        wheelRadius = p_meter;
    }
	
	void PIDController::setWheelTrack(const float p_meter)
	{
		wheelTrack = p_meter;
	}
    
    void PIDController::setPIDParameter(const minotaur_common::PIDParameter& p_param)
    {
        pidParameter = p_param;
    }
	
	void PIDController::setMouseSensorSettings(const MouseSensorSettings &p_mouseSettings)
	{
		mouseSettings = p_mouseSettings;
		mouseSensors.clear();
		for(int i = 0; i < mouseSettings.size(); ++i) {
			mouseSensors.push_back(pln_minotaur::PLN2033(mouseSettings[i].device));
			mouseSensors[i].setXResolution(mouseSettings[i].xResolution);
			mouseSensors[i].setXResolution(mouseSettings[i].yResolution);
		}
	}

    float PIDController::getLinearVelocity() const
	{
		//to get the formula see kinematic of two wheeled robots
		return (measuredVelocity.rightMPS + measuredVelocity.leftMPS) / 2;
	}
	
	float PIDController::getAngularVelocity() const
	{
		//to get the formula see kinematic of two wheeled robots
		return (measuredVelocity.rightMPS - measuredVelocity.leftMPS) / wheelTrack;
	}
    
    float PIDController::getWheelRadius() const
    {
        return wheelRadius;
    }
	
	float PIDController::getWheelTrack() const
	{
		return wheelTrack;
	}
    
     const minotaur_common::PIDParameter& PIDController::getPIDParameter() const
     {
         return pidParameter;
     }

    void PIDController::step(const int p_samplingIntervalMSec)
    {
        float samplingSec = msecToSec(p_samplingIntervalMSec);
        measureCurrentVelocity(samplingSec);
        
        calculateDifference();
        
        setMotorPower(samplingSec);
        
    }

    void PIDController::measureCurrentVelocity(const float p_samplingIntervalSecs)
    {
		// only use motors if no mouse sensors are available.
		// reading motors has high latency due to slow
		// ultrasonic sensors blocking the USB connection.
		if(mouseSettings.size() != 0)
			measuredVelocity = measureMouseVelocity(p_samplingIntervalSecs);
		else
			measuredVelocity = measureTickVelocity(p_samplingIntervalSecs);
        //TODO mouse velocity must be measured, currently only tickVel is used 
        //mouseVel = measureMouseVelocity();
    }

    MotorVelocity PIDController::measureTickVelocity(const float p_samplingIntervalSecs)
    {
        MotorVelocity result;
        float ticksPSLeft, ticksPSRight;
        
        //calculate ticks per second
        //samplingIntervall is needed, provided by the caller of step()
        //if the time is really correct must ensured by the caller
        ticksPSLeft =  ((float) leftMotor.getTachoInfo().blockTachoCount) / p_samplingIntervalSecs;
        ticksPSRight =  ((float) rightMotor.getTachoInfo().blockTachoCount) / p_samplingIntervalSecs;
        leftMotor.resetMotorPosition();
        rightMotor.resetMotorPosition();
        
        //convert ticks per second to meter per second
        result.set(ticksToMPS(ticksPSLeft), ticksToMPS(ticksPSRight));

        return result;
    }

    float PIDController::ticksToMPS(const float p_ticksPS)
    {
        float result;
        result = ((p_ticksPS * DEG_PER_TICK) / ((float) DEG_PER_CIRCLE)) * radiusToCircumference(wheelRadius);
        return result;
    }

    MotorVelocity PIDController::measureMouseVelocity(const float p_samplingIntervalSecs)
    {
		 MotorVelocity result;
		 result.leftMPS = 0;
		 result.rightMPS = 0;
		
		for(int i = 0; i < mouseSensors.size(); ++i) {
			double dx, dy;
			if(!mouseSensors[i].readStatusAndDisplacement(dx, dy)) {
				dx = 0;
				dy = 0;
			}
			Vector2 measurement, angularVec;
			
			measurement.x = cmToMeter(dx);
			measurement.y = cmToMeter(dy);
			
			measurement = rotateVec(measurement, mouseSettings[i].errorAngle);
			
			angularVec.x = (measurement.y * mouseSettings[i].y) / (2 * mouseSettings[i].x);
			angularVec.y = measurement.y;
			
			float angularVelocity = angularVec.length();
			float linearVelocity = measurement.x - angularVec.x;
			
			result.leftMPS += linearVelocity - (angularVelocity * wheelTrack) / 2;
			result.rightMPS += linearVelocity + (angularVelocity * wheelTrack) / 2;
		}
		
		result.leftMPS /= mouseSensors.size();
		result.rightMPS /= mouseSensors.size();
		
        return result;
    }

    void PIDController::calculateDifference()
    {
        //calculate difference between 'should be' (target)
        //and 'currently is' (measured) velocity
        lastDiff = currentDiff;
        currentDiff = targetVelocity - measuredVelocity;
        diffSum += currentDiff;
        
        //diffSum has to be limited to prevent type overflow
        if(diffSum.leftMPS > MAX_DIFF_SUM_MPS)
            diffSum.leftMPS = MAX_DIFF_SUM_MPS;
        else if( diffSum.leftMPS < MIN_DIFF_SUM_MPS)
            diffSum.leftMPS = MIN_DIFF_SUM_MPS;
            
        if(diffSum.rightMPS > MAX_DIFF_SUM_MPS)
            diffSum.rightMPS = MAX_DIFF_SUM_MPS;
        else if( diffSum.rightMPS < MIN_DIFF_SUM_MPS)
            diffSum.rightMPS = MIN_DIFF_SUM_MPS;
    }

    void PIDController::setMotorPower(const float p_samplingIntervalSecs)
    {
        //if motor should stay still, no PID-Controller is needed
        //just put power to 0
        if(targetVelocity.leftMPS != 0)
            powerLeft = pidMotorPower(currentDiff.leftMPS,
                                    lastDiff.leftMPS,
                                    diffSum.leftMPS,
                                    powerLeft,
                                    p_samplingIntervalSecs);
        else
            powerLeft = 0;
        
        if(targetVelocity.rightMPS != 0)
            powerRight = pidMotorPower(currentDiff.rightMPS,
                                    lastDiff.rightMPS,
                                    diffSum.rightMPS,
                                    powerRight,
                                    p_samplingIntervalSecs);
        else
            powerRight = 0;
        
        leftMotor.setPower(powerLeft);
        rightMotor.setPower(powerRight);
    }

    int PIDController::pidMotorPower(const float p_currentDiff,
                                    const float p_lastDiff,
                                    const float p_diffSum,
                                    const int p_motorPercent,
                                    const float p_samplingIntervalSecs)
    {
        int tmp, result;
        float u_motor;
        
        //calculate power factor via PID-Controller
        u_motor = pidParameter.Kp * p_currentDiff + pidParameter.Ki * p_samplingIntervalSecs * p_diffSum + pidParameter.Kd * (p_currentDiff - p_lastDiff) / p_samplingIntervalSecs;
        tmp = u_motor * MAX_MOTOR_POWER;

        //limit motorpower
        result = p_motorPercent + tmp;
        if(result > MAX_MOTOR_POWER)
            result = MAX_MOTOR_POWER;
        if(result < MIN_MOTOR_POWER)
            result = MIN_MOTOR_POWER;
        
        return result;
    }
}

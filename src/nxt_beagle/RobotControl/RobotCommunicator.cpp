#include "nxt_beagle/RobotCommunicator.hpp"
#include "nxt_beagle/Config.hpp"
#include "nxt_control/NxtOpcodes.hpp"

#define WHEEL_TRACK 0.12f
#define WHEEL_CIRCUMFERENCE 0.16f
#define LEFT_PORT PORT_A
#define RIGHT_PORT PORT_B

namespace minotaur
{
    RobotCommunicator::RobotCommunicator()
    {
        pthread_mutex_init(&robotMutex, NULL);
    }
    
    void RobotCommunicator::init(ros::NodeHandle &p_handle, nxtcon::Brick *p_brick)
    {
        leftMotor.setBrick(p_brick);
        leftMotor.setPort(LEFT_PORT);
        rightMotor.setBrick(p_brick);
        rightMotor.setPort(RIGHT_PORT);
        
        ROS_INFO("Setting up RobotController...");
        robotController.setWheelTrack(WHEEL_TRACK);
        robotController.getPIDController().setWheelCircumference(WHEEL_CIRCUMFERENCE);
        robotController.getPIDController().setLeftMotor(&leftMotor);
        robotController.getPIDController().setRightMotor(&rightMotor);
        
        ROS_INFO("Subscribing to topic \"%s\"...", NXT_SET_PID_PARAMETER);
        setPIDParamSub = p_handle.subscribe(NXT_SET_PID_PARAMETER, 1000, &RobotCommunicator::processPIDParamMsg, this);
        ROS_INFO("Subscribing to topic \"%s\"...", NXT_SET_ROBOT_VELOCITY_TOPIC);
        setRVelSub = p_handle.subscribe(NXT_SET_ROBOT_VELOCITY_TOPIC, 1000, &RobotCommunicator::processRobotVelocityMsg, this);
        
        ROS_INFO("Publishing on topic \"%s\"...", NXT_TARGET_MOTOR_VELOCITY_TOPIC);
        targetMVelPub = p_handle.advertise<nxt_beagle::MVelocity>(NXT_TARGET_MOTOR_VELOCITY_TOPIC, 1000);
        ROS_INFO("Publishing on topic \"%s\"...", NXT_MEASURE_MOTOR_VELOCITY_TOPIC);
        measuredMVelPub = p_handle.advertise<nxt_beagle::MVelocity>(NXT_MEASURE_MOTOR_VELOCITY_TOPIC, 1000);
        ROS_INFO("Publishing on topic \"%s\"...", NXT_TARGET_ROBOT_VELOCITY_TOPIC);
        targetRVelPub = p_handle.advertise<nxt_beagle::RVelocity>(NXT_TARGET_ROBOT_VELOCITY_TOPIC, 1000);
        ROS_INFO("Publishing on topic \"%s\"...", NXT_MEASURE_ROBOT_VELOCITY_TOPIC);
        measuredRVelPub = p_handle.advertise<nxt_beagle::RVelocity>(NXT_MEASURE_ROBOT_VELOCITY_TOPIC, 1000); 
    }
        
    void RobotCommunicator::publish()
    {
        nxt_beagle::MVelocity msgM;
        nxt_beagle::RVelocity msgR;
        
        if(pubTargetMVel)
        {
            msgM = motorVelocityToMsg(robotController.getPIDController().getVelocity());
            targetMVelPub.publish(msgM);
        }
        
        if(pubMeasuredMVel)
        {
            msgM = motorVelocityToMsg(robotController.getPIDController().getMeasuredVelocity());
            measuredMVelPub.publish(msgM);
        }
        
        if(pubTargetRVel)
        {
            msgR = robotVelocityToMsg(robotController.getRobotVelocity());
            targetRVelPub.publish(msgR);
        }
        
        if(pubMeasuredRVel)
        {
            msgR = robotVelocityToMsg(robotController.getMeasuredVelocity());
            measuredRVelPub.publish(msgR);
        }
    }
    
    nxt_beagle::MVelocity RobotCommunicator::motorVelocityToMsg(const minotaur::MotorVelocity& p_velocity)
    {
        nxt_beagle::MVelocity result;
        result.leftVelocity = p_velocity.leftMPS;
        result.rightVelocity = p_velocity.rightMPS;
        return result;
    }

    nxt_beagle::RVelocity RobotCommunicator::robotVelocityToMsg(const minotaur::RobotVelocity& p_velocity)
    {
        nxt_beagle::RVelocity result;
        result.linearVelocity = p_velocity.linearVelocity;
        result.angularVelocity = p_velocity.angularVelocity;
        return result;
    }
    
    RobotController& RobotCommunicator::getRobotController()
    {
        return robotController;
    }
    
    /* callbacks */
    void RobotCommunicator::processRobotVelocityMsg(const nxt_beagle::RVelocity& p_msg)
    {
        minotaur::RobotVelocity vel;
        vel.linearVelocity = p_msg.linearVelocity;
        vel.angularVelocity = p_msg.angularVelocity;
        
        lock();
        
        //always try catch between mutex lock / unlock to prevent deadlock
        try
        {
            robotController.setRobotVelocity(vel);
        }
        catch(std::exception const &e)
        {
            ROS_ERROR("SetRobotVelocity: %s.", e.what());
        }
        
        unlock();
    }
    
    void RobotCommunicator::processPIDParamMsg(const nxt_beagle::PIDParam& p_msg)
    {
        minotaur::PIDParameter params(p_msg.Kp, p_msg.Ki, p_msg.Kd);
        ROS_INFO("Changing PIDParameter to: Kp = %.4f; Ki = %.4f; Kd = %.4f.",params.Kp, params.Ki, params.Kd);  
        
        lock();
        //always try catch between mutex lock / unlock to prevent deadlock
        try
        {
            robotController.getPIDController().setPIDParameter(params);
        }
        catch(std::exception const &e)
        {
            ROS_ERROR("SetPIDParam: %s.", e.what());
        }
        
        unlock();
    }
}
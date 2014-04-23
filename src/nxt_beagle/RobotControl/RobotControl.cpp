/*
 * Author: Fabian Meyer 
 */

#include <ros/ros.h>
#include <pthread.h>
#include <signal.h>
#include <exception>
#include <string>
#include "nxt_beagle/Config.hpp"
#include "nxt_beagle/RobotController.hpp"
#include "nxt_beagle/SensorController.hpp"
#include "nxt_beagle/MVelocity.h"
#include "nxt_beagle/RVelocity.h"
#include "nxt_beagle/SamplingInterval.h"
#include "nxt_beagle/PIDParam.h"
#include "nxt_beagle/SetModel.h"
#include "nxt_beagle/nxtAddUltrasonic.h"
#include "nxt_beagle/nxtUltrasonic.h"
#include "nxt_beagle/ClearSensor.h"
#include "nxt_beagle/UltraSensor.h"

#include "nxt_control/Brick.hpp"
#include "nxt_control/NxtOpcodes.hpp"

#define NODE_NAME "RobotControl"
#define WHEEL_TRACK 0.12f
#define WHEEL_CIRCUMFERENCE 0.16f
#define DEF_SAMPLING_INTERVAL 100
#define LEFT_PORT PORT_A
#define RIGHT_PORT PORT_B

volatile int samplingMSec = DEF_SAMPLING_INTERVAL;
minotaur::RobotController robotController;
minotaur::SensorController sensorController;
bool publishTargetMVel = true;
bool publishMeasuredMvel = true;
bool publishTargetRVel = false;

nxtcon::Brick brick;
nxtcon::Motor leftMotor;
nxtcon::Motor rightMotor;

ros::Publisher targetMVelPub;
ros::Publisher measuredMVelPub;
ros::Publisher targetRVelPub;
ros::Publisher measuredRVelPub;
ros::Publisher sensorDataPub;

ros::Subscriber setRVelSub;
ros::Subscriber setSampIntSub;
ros::Subscriber setPIDParamSub;
ros::Subscriber setModelSub;
ros::Subscriber clearSensorSub;

ros::ServiceServer addUltraSonicSrv;
ros::ServiceServer getUltraSonicSrv;

pthread_mutex_t robotMutex = PTHREAD_MUTEX_INITIALIZER;
pthread_t thread;
volatile bool run = true;

/* Function prototypes */
bool init(ros::NodeHandle &p_handle);
void setSignals();
void signalHandler(int sig);
void processRobotVelocityMsg(const nxt_beagle::RVelocity &p_msg);
void processSamplingIntervallMsg(const nxt_beagle::SamplingInterval &p_msg) ;
void processPIDParamMsg(const nxt_beagle::PIDParam &p_msg);
void processSetModelMsg(const nxt_beagle::SetModel &p_msg);
void processClearSensorMsg(const nxt_beagle::ClearSensor &p_msg);
bool processGetUltrasonicRqt(nxt_beagle::nxtUltrasonic::Request  &req,
                             nxt_beagle::nxtUltrasonic::Response &res);
bool processAddUltrasonicRqt(nxt_beagle::nxtAddUltrasonic::Request  &req,
                             nxt_beagle::nxtAddUltrasonic::Response &res);
void* robotThread(void *arg);
void sendStatusInformation();
nxt_beagle::MVelocity motorVelocityToMsg(const minotaur::MotorVelocity& p_velocity);
nxt_beagle::RVelocity robotVelocityToMsg(const minotaur::RobotVelocity& p_velocity);

int main(int argc, char** argv)
{
    ros::init(argc, argv, NODE_NAME);
    ros::NodeHandle n;
    int ret;
    
    setSignals();
    
    
    if(!init(n))
    {
        ros::shutdown();
        return 1;
    }
    
    ROS_INFO("Starting RobotThread...");
    ret = pthread_create(&thread, NULL, robotThread, NULL);
    if(ret)
    {
        ROS_ERROR("Failed to create thread: %d", ret);
        ros::shutdown();
        return 2;
    }
    
    ROS_INFO("%s-System ready for input.", NODE_NAME);
    ros::spin();
    pthread_join(thread, NULL);
    
    return 0;
}

void setSignals()
{
    signal(SIGINT, signalHandler);
    signal(SIGQUIT, signalHandler);
    signal(SIGTERM, signalHandler);
}

void signalHandler(int sig)
{
    run = false;
}

bool init(ros::NodeHandle& p_handle)
{
    if(!ros::master::check())
    {
        ROS_ERROR("Roscore has to be started.");
        return false;
    }
    
    try
    {
        ROS_INFO("Initialize USBConnection to Brick...");
        brick.find();
        leftMotor.setBrick(&brick);
        leftMotor.setPort(LEFT_PORT);
        
        rightMotor.setBrick(&brick);
        rightMotor.setPort(RIGHT_PORT);
    }
    catch (std::exception const &e)
    {
        ROS_ERROR("Exception on initializing Brick: %s.", e.what());
        return false;
    }
    
    ROS_INFO("Publishing on topic \"%s\"...", NXT_TARGET_MOTOR_VELOCITY_TOPIC);
    targetMVelPub = p_handle.advertise<nxt_beagle::MVelocity>(NXT_TARGET_MOTOR_VELOCITY_TOPIC, 1000);
    ROS_INFO("Publishing on topic \"%s\"...", NXT_MEASURE_MOTOR_VELOCITY_TOPIC);
    measuredMVelPub = p_handle.advertise<nxt_beagle::MVelocity>(NXT_MEASURE_MOTOR_VELOCITY_TOPIC, 1000);
    
    ROS_INFO("Publishing on topic \"%s\"...", NXT_TARGET_ROBOT_VELOCITY_TOPIC);
    targetRVelPub = p_handle.advertise<nxt_beagle::RVelocity>(NXT_TARGET_ROBOT_VELOCITY_TOPIC, 1000);
    ROS_INFO("Publishing on topic \"%s\"...", NXT_MEASURE_ROBOT_VELOCITY_TOPIC);
    measuredRVelPub = p_handle.advertise<nxt_beagle::RVelocity>(NXT_MEASURE_ROBOT_VELOCITY_TOPIC, 1000); 
    
    ROS_INFO("Publishing on topic \"%s\"...", NXT_ULTRA_SENSOR_TOPIC);
    sensorDataPub = p_handle.advertise<nxt_beagle::UltraSensor>(NXT_ULTRA_SENSOR_TOPIC, 1000); 
    
    ROS_INFO("Subscribing to topic \"%s\"...", NXT_SET_ROBOT_VELOCITY_TOPIC);
    setRVelSub = p_handle.subscribe(NXT_SET_ROBOT_VELOCITY_TOPIC, 1000, processRobotVelocityMsg);
    ROS_INFO("Subscribing to topic \"%s\"...", NXT_SET_SAMPLING_INTERVAL_TOPIC);
    setSampIntSub = p_handle.subscribe(NXT_SET_SAMPLING_INTERVAL_TOPIC, 1000, processSamplingIntervallMsg);
    ROS_INFO("Subscribing to topic \"%s\"...", NXT_SET_PID_PARAMETER);
    setPIDParamSub = p_handle.subscribe(NXT_SET_PID_PARAMETER, 1000, processPIDParamMsg);
    ROS_INFO("Subscribing to topic \"%s\"...", NXT_SET_MODEL_TOPIC);
    setModelSub = p_handle.subscribe(NXT_SET_MODEL_TOPIC, 1000, processSetModelMsg);
    ROS_INFO("Subscribing to topic \"%s\"...", NXT_CLEAR_SENSOR_TOPIC);
    clearSensorSub = p_handle.subscribe(NXT_CLEAR_SENSOR_TOPIC, 1000, processClearSensorMsg);
    
    ROS_INFO("Offering service \"%s\"...", NXT_GET_ULTRASONIC_SRV);
    getUltraSonicSrv = p_handle.advertiseService(NXT_GET_ULTRASONIC_SRV, processGetUltrasonicRqt);
    ROS_INFO("Offering service \"%s\"...", NXT_ADD_ULTRASONIC_SRV);
    addUltraSonicSrv = p_handle.advertiseService(NXT_ADD_ULTRASONIC_SRV, processAddUltrasonicRqt);
    
    ROS_INFO("Setting up RobotController...");
    robotController.setWheelTrack(WHEEL_TRACK);
    robotController.getPIDController().setWheelCircumference(WHEEL_CIRCUMFERENCE);
    robotController.getPIDController().setLeftMotor(&leftMotor);
    robotController.getPIDController().setRightMotor(&rightMotor);
    
    ROS_INFO("Setting up SensorController...");
    sensorController.setBrick(&brick);
    
    return true;
}

void processRobotVelocityMsg(const nxt_beagle::RVelocity& p_msg)
{
    minotaur::RobotVelocity vel;
    vel.linearVelocity = p_msg.linearVelocity;
    vel.angularVelocity = p_msg.angularVelocity;
    
    pthread_mutex_lock(&robotMutex);
    
    robotController.setRobotVelocity(vel);
    
    pthread_mutex_unlock(&robotMutex);
}

void processSamplingIntervallMsg(const nxt_beagle::SamplingInterval& p_msg) 
{
    ROS_INFO("Sampling Interval changed to %d msec.", p_msg.msec);
    pthread_mutex_lock(&robotMutex);
    
    samplingMSec = p_msg.msec;
    
    pthread_mutex_unlock(&robotMutex);
}

void processPIDParamMsg(const nxt_beagle::PIDParam& p_msg)
{
    minotaur::PIDParameter params(p_msg.Kp, p_msg.Ki, p_msg.Kd);
    ROS_INFO("PIDParameter changed to: Kp = %.4f; Ki = %.4f; Kd = %.4f.",params.Kp, params.Ki, params.Kd);
             
    pthread_mutex_lock(&robotMutex);
    
    robotController.getPIDController().setPIDParameter(params);
    
    pthread_mutex_unlock(&robotMutex);
}

void processSetModelMsg(const nxt_beagle::SetModel& p_msg)
{
    ROS_INFO("Set robot model \"%s\"...", p_msg.name.c_str());
    int samplingTmp, hadError = 0;
    float wheelTrack, wheelCircumference;
    minotaur::PIDParameter pidParams;
    
    //read parameter from ros parameter server
    if(!ros::param::get(PARAM_WHEEL_TRACK(p_msg.name), wheelTrack))
    {
        ROS_ERROR("SetModel: Could not read param \"%s\".", PARAM_WHEEL_TRACK(p_msg.name).c_str());
        hadError = 1;
    }
    
    if(!ros::param::get(PARAM_WHEEL_CIRCUMFERENCE(p_msg.name), wheelCircumference))
    {
        ROS_ERROR("SetModel: Could not read param \"%s\".", PARAM_WHEEL_CIRCUMFERENCE(p_msg.name).c_str());
        hadError = 1;
    }
    
    if(!ros::param::get(PARAM_KP(p_msg.name), pidParams.Kp))
    {
        ROS_ERROR("SetModel: Could not read param \"%s\".", PARAM_KP(p_msg.name).c_str());
        hadError = 1;
    }
    
    if(!ros::param::get(PARAM_KI(p_msg.name), pidParams.Ki))
    {
        ROS_ERROR("SetModel: Could not read param \"%s\".", PARAM_KI(p_msg.name).c_str());
        hadError = 1;
    }
    
    if(!ros::param::get(PARAM_KD(p_msg.name), pidParams.Kd))
    {
        ROS_ERROR("SetModel: Could not read param \"%s\".", PARAM_KD(p_msg.name).c_str());
        hadError = 1;
    }
    
    if(!ros::param::get(PARAM_SAMPLING_INTERVAL(p_msg.name), samplingTmp))
    {
        ROS_ERROR("SetModel: Could not read param \"%s\".", PARAM_SAMPLING_INTERVAL(p_msg.name).c_str());
        hadError = 1;
    }
    
    if(!hadError)
    {
        ROS_INFO("Track = %.2f m; Circumference = %.2f m; Interval = %d ms", wheelTrack, wheelCircumference, samplingTmp);
        ROS_INFO("Kp = %.2f; Ki = %.2f; Kd = %.2f", pidParams.Kp, pidParams.Ki, pidParams.Kd);
        
        pthread_mutex_lock(&robotMutex);
        
        robotController.getPIDController().setPIDParameter(pidParams);
        robotController.getPIDController().setWheelCircumference(wheelCircumference);
        robotController.setWheelTrack(wheelTrack);
        samplingMSec = samplingTmp;
        
        pthread_mutex_unlock(&robotMutex);
    }
}

void processClearSensorMsg(const nxt_beagle::ClearSensor &p_msg)
{
    sensorController.clearSensors();
}

bool processGetUltrasonicRqt(nxt_beagle::nxtUltrasonic::Request  &req,
                             nxt_beagle::nxtUltrasonic::Response &res)
{
    res.distance = sensorController.getDistance(req.sensorID);
    return true;
}

bool processAddUltrasonicRqt(nxt_beagle::nxtAddUltrasonic::Request  &req,
                             nxt_beagle::nxtAddUltrasonic::Response &res)
{
    uint8_t tmpPort;
    switch(req.port)
    {
        case NXT_PORT1:
            tmpPort = PORT_1;
            break;
        case NXT_PORT2:
            tmpPort = PORT_2;
            break;
        case NXT_PORT3:
            tmpPort = PORT_3;
            break;
        case NXT_PORT4:
            tmpPort = PORT_4;
            break;
        default:
            ROS_ERROR("Cannot add UltrasonicSensor. Invalid sensor port: %d.", req.port);
            return false;
    }
    
    try
    {
        res.sensorID = sensorController.addSensor(tmpPort);
    }
    catch(std::exception const &e)
    {
        ROS_ERROR("Cannot add UltrasonicSensor: %s.", e.what());
        return false;
    }
    return true;
}

void* robotThread(void *arg)
{
    ros::Time begin, end;
    float sleepSec;
    
    ROS_INFO("RobotThread started.");
    
    while(run)
    {
        begin = ros::Time::now();
        
        //lock robotMutex
        pthread_mutex_lock(&robotMutex);
        
        try
        {
            robotController.step(samplingMSec);
            //TODO is this the right place to send infos, maybe bad for performance
            //better to do it asynch?
            sendStatusInformation();
            
            end = ros::Time::now();
            
            sleepSec = MS_TO_SEC(samplingMSec) - (end - begin).toSec();
        }
        catch(std::exception const & e)
        {
            ROS_ERROR("RobotThread: %s.", e.what());
        }
        //unlock robotMutex
        pthread_mutex_unlock(&robotMutex);
        
        if(sleepSec > 0)
            ros::Duration(sleepSec).sleep();
    }
    ROS_INFO("RobotThread terminated.");
    ros::shutdown();
}

void sendStatusInformation()
{
    nxt_beagle::MVelocity msgM;
    nxt_beagle::RVelocity msgR;
    
    if(publishTargetMVel)
    {
        msgM = motorVelocityToMsg(robotController.getPIDController().getVelocity());
        targetMVelPub.publish(msgM);
    }
    
    if(publishMeasuredMvel)
    {
        msgM = motorVelocityToMsg(robotController.getPIDController().getMeasuredVelocity());
        ROS_INFO("Sending Measured Velocity: left = &.2f; right = %.2f;", result.leftVelocity, result.rightVelocity);
        measuredMVelPub.publish(msgM);
    }
    
    if(publishTargetRVel)
    {
        msgR = robotVelocityToMsg(robotController.getRobotVelocity());
        targetRVelPub.publish(msgR);
    }
}

nxt_beagle::MVelocity motorVelocityToMsg(const minotaur::MotorVelocity& p_velocity)
{
    nxt_beagle::MVelocity result;
    result.leftVelocity = p_velocity.leftMPS;
    result.rightVelocity = p_velocity.rightMPS;
    return result;
}

nxt_beagle::RVelocity robotVelocityToMsg(const minotaur::RobotVelocity& p_velocity)
{
    nxt_beagle::RVelocity result;
    result.linearVelocity = p_velocity.linearVelocity;
    result.angularVelocity = p_velocity.angularVelocity;
    return result;
}

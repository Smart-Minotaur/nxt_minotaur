#include <ros/ros.h>
#include <pthread.h>
#include <signal.h>
#include "nxt_beagle/Config.hpp"
#include "nxt_beagle/RobotController.hpp"
#include "nxt_beagle/MVelocity.h"
#include "nxt_beagle/RVelocity.h"
#include "nxt_beagle/SamplingInterval.h"
#include "nxt_beagle/PIDParam.h"
#include "nxt_beagle/nxtPower.h"
#include "nxt_beagle/nxtTicks.h"

#define NODE_NAME "RobotControl"
#define WHEEL_TRACK HERACLES_WHEEL_TRACK
#define WHEEL_CIRCUMFERENCE HERACLES_WHEEL_CIRCUMFERENCE
#define DEF_SAMPLING_INTERVAL 0.1f

volatile float samplingSec = DEF_SAMPLING_INTERVAL;
minotaur::RobotController robotController;
bool publishTargetMVel = true;
bool publishMeasuredMvel = true;
bool publishTargetRVel = false;

ros::Publisher targetMVelPub;
ros::Publisher measuredMVelPub;
ros::Publisher targetRVelPub;
ros::Publisher measuredRVelPub;
ros::Publisher powerPublisher;

ros::Subscriber setRVelSub;
ros::Subscriber setSampIntSub;
ros::Subscriber setPIDParamSub;

ros::ServiceClient tickClient;

pthread_mutex_t robotMutex = PTHREAD_MUTEX_INITIALIZER;
pthread_t thread;
volatile bool run = true;

/* Function prototypes */
bool init(ros::NodeHandle& p_handle);
void setSignals();
void signalHandler(int sig);
void processRobotVelocityMsg(const nxt_beagle::RVelocity& p_msg);
void processSamplingIntervallMsg(const nxt_beagle::SamplingInterval& p_msg) ;
void processPIDParamMsg(const nxt_beagle::PIDParam& p_msg);
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
        return -1;
    
    ROS_INFO("Starting RobotThread...");
    ret = pthread_create(&thread, NULL, robotThread, NULL);
    if(ret)
    {
        ROS_ERROR("Failed to create thread: %d", ret);
        return -2;
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
    
    ROS_INFO("Publishing on topic \"%s\"...", NXT_TARGET_MOTOR_VELOCITY_TOPIC);
    targetMVelPub = p_handle.advertise<nxt_beagle::MVelocity>(NXT_TARGET_MOTOR_VELOCITY_TOPIC, 1000);
    ROS_INFO("Publishing on topic \"%s\"...", NXT_MEASURE_MOTOR_VELOCITY_TOPIC);
    measuredMVelPub = p_handle.advertise<nxt_beagle::MVelocity>(NXT_MEASURE_MOTOR_VELOCITY_TOPIC, 1000);
    
    ROS_INFO("Publishing on topic \"%s\"...", NXT_TARGET_ROBOT_VELOCITY_TOPIC);
    targetRVelPub = p_handle.advertise<nxt_beagle::RVelocity>(NXT_TARGET_ROBOT_VELOCITY_TOPIC, 1000);
    ROS_INFO("Publishing on topic \"%s\"...", NXT_MEASURE_ROBOT_VELOCITY_TOPIC);
    measuredRVelPub = p_handle.advertise<nxt_beagle::RVelocity>(NXT_MEASURE_ROBOT_VELOCITY_TOPIC, 1000); 
    
    ROS_INFO("Publishing on topic \"%s\"...", NXT_POWER_TOPIC);
    powerPublisher = p_handle.advertise<nxt_beagle::nxtPower>(NXT_POWER_TOPIC, 1000);
    
    ROS_INFO("Subscribing to topic \"%s\"...", NXT_SET_ROBOT_VELOCITY_TOPIC);
    setRVelSub = p_handle.subscribe(NXT_SET_ROBOT_VELOCITY_TOPIC, 1000, processRobotVelocityMsg);
    ROS_INFO("Subscribing to topic \"%s\"...", NXT_SET_SAMPLING_INTERVAL_TOPIC);
    setSampIntSub = p_handle.subscribe(NXT_SET_SAMPLING_INTERVAL_TOPIC, 1000, processSamplingIntervallMsg);
    ROS_INFO("Subscribing to topic \"%s\"...", NXT_SET_PID_PARAMETER);
    setPIDParamSub = p_handle.subscribe(NXT_SET_PID_PARAMETER, 1000, processPIDParamMsg);
    
    ROS_INFO("Using Servics \"%s\"...", NXT_GET_TICKS_SRV);
    tickClient = p_handle.serviceClient<nxt_beagle::nxtTicks>(NXT_GET_TICKS_SRV);
    
    ROS_INFO("Setting up RobotController...");
    robotController.setWheelTrack(WHEEL_TRACK);
    robotController.getPIDController().setWheelCircumference(WHEEL_CIRCUMFERENCE);
    robotController.getPIDController().setMotorPublisher(&powerPublisher);
    robotController.getPIDController().setMotorClient(&tickClient);
    
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
    pthread_mutex_lock(&robotMutex);
    
    samplingSec = p_msg.sec;
    
    pthread_mutex_unlock(&robotMutex);
}

void processPIDParamMsg(const nxt_beagle::PIDParam& p_msg)
{
    minotaur::PIDParameter params(p_msg.Kp, p_msg.Ki, p_msg.Kd);
    
    pthread_mutex_lock(&robotMutex);
    
    robotController.getPIDController().setPIDParameter(params);
    
    pthread_mutex_unlock(&robotMutex);
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
        
        robotController.step(samplingSec);
        //TODO is this the right place to send infos, maybe bad for performance
        //better to do it asynch?
        sendStatusInformation();
        
        end = ros::Time::now();
        
        sleepSec = samplingSec - (end - begin).toSec();
        
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

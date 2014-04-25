/*
 * Author: Fabian Meyer 
 */

#include <ros/ros.h>
#include <pthread.h>
#include <signal.h>
#include <exception>
#include <string>
#include "nxt_beagle/Config.hpp"
#include "nxt_beagle/SamplingInterval.h"
#include "nxt_beagle/SetModel.h"
#include "nxt_control/Brick.hpp"
#include "nxt_control/NxtOpcodes.hpp"
#include "nxt_beagle/RobotCommunicator.hpp"
#include "nxt_beagle/SensorCommunicator.hpp"

#define NODE_NAME "RobotControl"
#define WHEEL_TRACK 0.12f
#define WHEEL_CIRCUMFERENCE 0.16f
#define DEF_SAMPLING_INTERVAL 100
#define LEFT_PORT PORT_A
#define RIGHT_PORT PORT_B

volatile int samplingMSec = DEF_SAMPLING_INTERVAL;

nxtcon::Brick brick;
minotaur::RobotCommunicator robotCommunicator;
minotaur::SensorCommunicator sensorCommunicator;

ros::Subscriber setSampIntSub;
ros::Subscriber setModelSub;

pthread_mutex_t intervalMutex = PTHREAD_MUTEX_INITIALIZER;
pthread_t robot_thread;
pthread_t sensor_thread;

pthread_barrier_t startBarrier;
volatile bool run = true;

/* Function prototypes */
void setSignals();
void signalHandler(int sig);
bool init(ros::NodeHandle &p_handle);
bool startThreads();
void* robotThread(void *arg);
void* sensorThread(void *arg);
void joinThreads();
void processSamplingIntervallMsg(const nxt_beagle::SamplingInterval &p_msg) ;
void processSetModelMsg(const nxt_beagle::SetModel &p_msg);

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
    
    if(!startThreads())
    {
        ros::shutdown();
        return 2;
    }
    ROS_INFO("%s-System ready for input.", NODE_NAME);
    
    ros::spin();
    joinThreads();
    
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
    int ret;
    
    if(!ros::master::check())
    {
        ROS_ERROR("Roscore has to be started.");
        return false;
    }
    
    try
    {
        ROS_INFO("Initialize USBConnection to Brick...");
        brick.find();
    }
    catch (std::exception const &e)
    {
        ROS_ERROR("InitializeBrick: %s.", e.what());
        return false;
    }
    
    ret = pthread_barrier_init(&startBarrier, NULL, 3);
    if(ret)
    {
        ROS_ERROR("Could not init startBarrier: %d.", ret);
        return false;
    }
    
    try
    {
        robotCommunicator.pubTargetMVel = true;
        robotCommunicator.pubMeasuredMVel = true;
        robotCommunicator.pubTargetRVel = false;
        robotCommunicator.pubMeasuredRVel = false;
        
        robotCommunicator.init(p_handle, &brick);
        sensorCommunicator.init(p_handle, &brick);
    }
    catch(std::exception const &e)
    {
        ROS_ERROR("Initialize Communicator: %s.", e.what());
        return false;
    }
    
    ROS_INFO("Subscribing to topic \"%s\"...", NXT_SET_SAMPLING_INTERVAL_TOPIC);
    setSampIntSub = p_handle.subscribe(NXT_SET_SAMPLING_INTERVAL_TOPIC, 1000, processSamplingIntervallMsg);
    ROS_INFO("Subscribing to topic \"%s\"...", NXT_SET_MODEL_TOPIC);
    setModelSub = p_handle.subscribe(NXT_SET_MODEL_TOPIC, 1000, processSetModelMsg);
    
    return true;
}

bool startThreads()
{
    int ret;
    
    ROS_INFO("Starting RobotThread...");
    
    ret = pthread_create(&robot_thread, NULL, robotThread, NULL);
    if(ret)
    {
        ROS_ERROR("Failed to create thread: %d", ret);
        return false;
    }
    
    ROS_INFO("Starting SensorThread...");
    
    ret = pthread_create(&sensor_thread, NULL, sensorThread, NULL);
    if(ret)
    {
        ROS_ERROR("Failed to create thread: %d", ret);
        return false;
    }
    
    pthread_barrier_wait(&startBarrier);
    
    return true;
}

void* robotThread(void *arg)
{
    ros::Time begin, end;
    float sleepSec;
    int tmpSamplingMsec;
    
    ROS_INFO("RobotThread started.");
    
    pthread_barrier_wait(&startBarrier);
    
    while(run)
    {
        sleepSec = 0;
        begin = ros::Time::now();
        
        //lock 
        pthread_mutex_lock(&intervalMutex);
        tmpSamplingMsec = samplingMSec;
        pthread_mutex_unlock(&intervalMutex);
        
        robotCommunicator.lock();
        try
        {
            
            robotCommunicator.getRobotController().step(tmpSamplingMsec);
            //TODO is this the right place to send infos, maybe bad for performance
            //better to do it asynch?
            robotCommunicator.publish();
            
            end = ros::Time::now();
            
            sleepSec = MS_TO_SEC(tmpSamplingMsec) - (end - begin).toSec();
        }
        catch(std::exception const & e)
        {
            ROS_ERROR("RobotThread: %s.", e.what());
        }
        
        robotCommunicator.unlock();
        if(sleepSec > 0)
            ros::Duration(sleepSec).sleep();
    }
    
    ROS_INFO("RobotThread terminated.");
    ros::shutdown();
}

void* sensorThread(void *arg)
{
    ros::Time begin, end;
    float sleepSec;
    int tmpSamplingMsec;
    
    ROS_INFO("SensorThread started.");
    
    pthread_barrier_wait(&startBarrier);
    
    while(run)
    {
        
        sleepSec = 0;
        begin = ros::Time::now();
        
        pthread_mutex_lock(&intervalMutex);
        tmpSamplingMsec = samplingMSec;
        pthread_mutex_unlock(&intervalMutex);
        
        sensorCommunicator.lock();
        
        try
        {
            sensorCommunicator.publish();
        }
        catch(std::exception const &e)
        {
            ROS_ERROR("SensorThread: %s.", e.what());
        }
        sensorCommunicator.unlock();
        
        if(sleepSec > 0)
            ros::Duration(sleepSec).sleep();
    }
    
    ROS_INFO("SensorThread terminated.");
}

void joinThreads()
{
    pthread_join(robot_thread, NULL);
    pthread_join(sensor_thread, NULL);
}

void processSamplingIntervallMsg(const nxt_beagle::SamplingInterval& p_msg) 
{
    ROS_INFO("Sampling Interval changed to %d msec.", p_msg.msec);
    pthread_mutex_lock(&intervalMutex);
    
    samplingMSec = p_msg.msec;
    
    pthread_mutex_unlock(&intervalMutex);
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
        
        pthread_mutex_lock(&intervalMutex);
        robotCommunicator.lock();
        //always try catch between mutex lock / unlock to prevent deadlock
        try
        {
        robotCommunicator.getRobotController().getPIDController().setPIDParameter(pidParams);
        robotCommunicator.getRobotController().getPIDController().setWheelCircumference(wheelCircumference);
        robotCommunicator.getRobotController().setWheelTrack(wheelTrack);
        samplingMSec = samplingTmp;
        }
        catch(std::exception const &e)
        {
            ROS_ERROR("SetModel: %s.", e.what());
        }
        robotCommunicator.unlock();
        pthread_mutex_unlock(&intervalMutex);
    }
}
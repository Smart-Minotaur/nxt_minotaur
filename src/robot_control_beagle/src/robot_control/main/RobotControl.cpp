/*
 * Author: Fabian Meyer 
 */

#include <ros/ros.h>
#include <pthread.h>
#include <signal.h>
#include <exception>
#include <string>
#include "robot_control_beagle/Utils.hpp"
#include "nxt_control/Brick.hpp"
#include "nxt_control/NxtOpcodes.hpp"
#include "nxt_control/NxtExceptions.hpp"
#include "robot_control_beagle/RobotCommunicator.hpp"
#include "robot_control_beagle/SensorCommunicator.hpp"

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

pthread_mutex_t intervalMutex = PTHREAD_MUTEX_INITIALIZER;
pthread_t robot_thread;
pthread_t sensor_thread;

pthread_barrier_t startBarrier;
pthread_barrier_t stopBarrier;
volatile bool run = true;

/* Function prototypes */
void setSignals();
void signalHandler(int sig);
void shutdown();
bool init(ros::NodeHandle &p_handle, tf::TransformBroadcaster *p_broadcaster);
bool selectModel();
bool setModel(const std::string& p_name);
bool setModelSensors(const int p_sensorCount);
bool startThreads();
void* robotThread(void *arg);
void* sensorThread(void *arg);
void joinThreads();

int main(int argc, char** argv)
{
    ros::init(argc, argv, NODE_NAME);
    ros::NodeHandle n;
    tf::TransformBroadcaster broadcaster;
    int ret;
    
    setSignals();
    
    
    if(!init(n, &broadcaster))
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
    shutdown();
}

void shutdown()
{
    run = false;
}

bool init(ros::NodeHandle& p_handle, tf::TransformBroadcaster *p_broadcaster)
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
    
    ret = pthread_barrier_init(&stopBarrier, NULL, 2);
    if(ret)
    {
        ROS_ERROR("Could not init stopBarrier: %d.", ret);
        return false;
    }
    
    try
    {       
        robotCommunicator.setTransformBroadcaster(p_broadcaster);
        robotCommunicator.init(p_handle, &brick);
        sensorCommunicator.init(p_handle, &brick);
    }
    catch(std::exception const &e)
    {
        ROS_ERROR("Initialize Communicator: %s.", e.what());
        return false;
    }
    
    if(!selectModel())
        return false;
    
    return true;
}

bool selectModel()
{   
    std::string model;
    if(!ros::param::get(PARAM_CURRENT_MODEL(),model))
    {
        ROS_ERROR("Could not get CurrentModel from param-server.");
        return false;
    }
    
    ROS_INFO("Selecting \"%s\" as model...", model.c_str());
    setModel(model);
    
    return true;
}

bool setModel(const std::string& p_name)
{
    ROS_INFO("Set robot model \"%s\"...", p_name.c_str());
    int samplingTmp;
    bool hadError = false;
    float wheelTrack, wheelCircumference;
    minotaur::PIDParameter pidParams;
    
    //read parameter from ros parameter server
    if(!ros::param::get(PARAM_WHEEL_TRACK(p_name), wheelTrack))
    {
        ROS_ERROR("SetModel: Could not read param \"%s\".", PARAM_WHEEL_TRACK(p_name).c_str());
        hadError = true;
    }
    
    if(!ros::param::get(PARAM_WHEEL_CIRCUMFERENCE(p_name), wheelCircumference))
    {
        ROS_ERROR("SetModel: Could not read param \"%s\".", PARAM_WHEEL_CIRCUMFERENCE(p_name).c_str());
        hadError = true;
    }
    
    if(!ros::param::get(PARAM_KP(p_name), pidParams.Kp))
    {
        ROS_ERROR("SetModel: Could not read param \"%s\".", PARAM_KP(p_name).c_str());
        hadError = true;
    }
    
    if(!ros::param::get(PARAM_KI(p_name), pidParams.Ki))
    {
        ROS_ERROR("SetModel: Could not read param \"%s\".", PARAM_KI(p_name).c_str());
        hadError = true;
    }
    
    if(!ros::param::get(PARAM_KD(p_name), pidParams.Kd))
    {
        ROS_ERROR("SetModel: Could not read param \"%s\".", PARAM_KD(p_name).c_str());
        hadError = true;
    }
    
    if(!ros::param::get(PARAM_SAMPLING_INTERVAL(p_name), samplingTmp))
    {
        ROS_ERROR("SetModel: Could not read param \"%s\".", PARAM_SAMPLING_INTERVAL(p_name).c_str());
        hadError = true;
    }
    
    int sensorCount = 0;
    while(true)
    {
        if(!ros::param::has(PARAM_SENSOR(p_name, sensorCount)))
            break;
        
        ++sensorCount;
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
        
        hadError = setModelSensors(sensorCount);
    }
    
    return hadError;
}

bool setModelSensors(const int p_sensorCount)
{
    bool result = true;
    sensorCommunicator.lock();
    try
    {
        sensorCommunicator.getSensorController().clearSensors();
        for(int i = 0; i < p_sensorCount; ++i) {
            ROS_INFO("Adding sensor %d...", i);
            uint8_t tmpPort;
            switch(i)
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
                    ROS_ERROR("SetModelSensors: Invalid sensor port: %d.", i);
                    return false;
            }
            sensorCommunicator.getSensorController().addSensor(tmpPort);
        }
        
    }
    catch(std::exception const &e)
    {
        ROS_ERROR("Failed to add sensors to SensorController : %s.", e.what());
        result = false;
    }
    sensorCommunicator.unlock();
    
    return result;
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
        }
        catch(nxtcon::USBException const &ue)
        {
            ROS_ERROR("RobotThread: %s.", ue.what());
            shutdown();
        }
        catch(std::exception const & e)
        {
            ROS_WARN("RobotThread: %s.", e.what());
        }
        
        robotCommunicator.unlock();
        
        end = ros::Time::now();
        sleepSec = MS_TO_SEC(tmpSamplingMsec) - (end - begin).toSec();
        
        if(sleepSec > 0)
            ros::Duration(sleepSec).sleep();
    }
    
    ROS_INFO("RobotThread terminated.");
    pthread_barrier_wait(&stopBarrier);
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
        catch(nxtcon::USBException const &ue)
        {
            ROS_ERROR("SensorThread: %s.", ue.what());
            shutdown();
        }
        catch(std::exception const &e)
        {
            ROS_WARN("SensorThread: %s.", e.what());
        }
        sensorCommunicator.unlock();
        
        end = ros::Time::now();
        sleepSec = MS_TO_SEC(tmpSamplingMsec) - (end - begin).toSec();
        
        if(sleepSec > 0)
            ros::Duration(sleepSec).sleep();
    }
    
    ROS_INFO("SensorThread terminated.");
    pthread_barrier_wait(&stopBarrier);
}

void joinThreads()
{
    pthread_join(robot_thread, NULL);
    pthread_join(sensor_thread, NULL);
}
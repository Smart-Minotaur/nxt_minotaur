#include <ros/ros.h>
#include <pthread.h>
#include <signal.h>
#include <exception>
#include <string>
#include "nxt_control/Brick.hpp"
#include "nxt_control/NxtOpcodes.hpp"
#include "nxt_control/NxtExceptions.hpp"
#include "robot_control/RobotCommunicator.hpp"
#include "robot_control/SensorCommunicator.hpp"
#include "minotaur_common/RAIILock.hpp"
#include "minotaur_common/SensorSettings.hpp"
#include "minotaur_common/RobotSettings.hpp"
#include "minotaur_common/Math.hpp"

#define NODE_NAME "RobotControl"
#define WHEEL_TRACK 0.12f
#define WHEEL_RADIUS 0.025f
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
bool selectCurrentModel();
bool selectModel(const std::string& p_name);
bool applySettings(const RobotSettings &p_robotSettings, const std::vector<SensorSetting> &p_sensorSettings);
bool setModelSensors(const int p_sensorCount);
bool startThreads();
void* robotThread(void *arg);
void stepRobotController();
void* sensorThread(void *arg);
void publishSensorData();
void joinThreads();
int getSamplingInterval();
void setSamplingInterval(const int p_intervalMsec);

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
    
    if(!ros::master::check()) {
        ROS_ERROR("Roscore has to be started.");
        return false;
    }
    
    try
    {
        ROS_INFO("Initialize USBConnection to Brick...");
        brick.find();
    } catch (const std::exception &e) {
        ROS_ERROR("InitializeBrick: %s.", e.what());
        return false;
    }
    
    ret = pthread_barrier_init(&startBarrier, NULL, 3);
    if(ret) {
        ROS_ERROR("Could not init startBarrier: %d.", ret);
        return false;
    }
    
    ret = pthread_barrier_init(&stopBarrier, NULL, 2);
    if(ret) {
        ROS_ERROR("Could not init stopBarrier: %d.", ret);
        return false;
    }
    
    try {       
        robotCommunicator.setTransformBroadcaster(p_broadcaster);
        robotCommunicator.init(p_handle, &brick);
        sensorCommunicator.init(p_handle, &brick);
    } catch(const std::exception &e) {
        ROS_ERROR("Initialize Communicator: %s.", e.what());
        return false;
    }
    
    if(!selectCurrentModel())
        return false;
    
    return true;
}

bool selectCurrentModel()
{   
    RobotSettings robotSettings;
    std::vector<SensorSetting> sensorSettings;
    
    ROS_INFO("Select current model...");
    try {
        robotSettings.loadCurrentFromParamServer();
        sensorSettings = loadCurrentSensorSettings();
    } catch (const std::exception &e) {
        ROS_ERROR("LoadCurrentModel: %s.", e.what());
        return false;
    }
    ROS_INFO("Found model \"%s\".", robotSettings.modelName.c_str());
    
    return applySettings(robotSettings, sensorSettings);
}

bool selectModel(const std::string& p_name)
{
    ROS_INFO("Set robot model \"%s\"...", p_name.c_str());
    
    RobotSettings robotSettings;
    std::vector<SensorSetting> sensorSettings;
    try {
        robotSettings.loadFromParamServer(p_name);
        sensorSettings = loadSensorSettings(p_name);
    } catch (const std::exception &e) {
        ROS_ERROR("LoadModel: %s.", e.what());
        return false;
    }
    
    return applySettings(robotSettings, sensorSettings);
}

bool applySettings(const RobotSettings &p_robotSettings, const std::vector<SensorSetting> &p_sensorSettings)
{
    ROS_INFO("Track = %.2f m; Circumference = %.2f m; Interval = %d ms", wheelTrack, wheelCircumference, samplingTmp);
    ROS_INFO("Kp = %.2f; Ki = %.2f; Kd = %.2f", pidParams.Kp, pidParams.Ki, pidParams.Kd);
    
    RAIILock lock1(&intervalMutex);
    RAIILock lock2(robotCommunicator.mutex());
    try {
        robotCommunicator.getRobotController().getPIDController().setPIDParameter(p_robotSettings.pidParameter);
        robotCommunicator.getRobotController().getPIDController().setWheelRadius(p_robotSettings.wheelRadius);
        robotCommunicator.getRobotController().setWheelTrack(p_robotSettings.wheelTrack);
        samplingMSec = p_robotSettings.samplingInterval;
    } catch(const std::exception &e) {
        ROS_ERROR("ApplySettings: %s.", e.what());
        return false;
    } catch(...) {
        ROS_ERROR("ApplySettings: Caught unkonwn error.");
        return false;
    }
    
    return setModelSensors(p_sensorSettings.size());
}

bool setModelSensors(const int p_sensorCount)
{
    RAIILock lock(sensorCommunicator.mutex());
    try {
        sensorCommunicator.getSensorController().clearSensors();
        for(int i = 0; i < p_sensorCount; ++i) {
            ROS_INFO("Adding sensor %d...", i);
            uint8_t tmpPort;
            switch(i) {
                case MINOTAUR_PORT1:
                    tmpPort = PORT_1;
                    break;
                case MINOTAUR_PORT2:
                    tmpPort = PORT_2;
                    break;
                case MINOTAUR_PORT3:
                    tmpPort = PORT_3;
                    break;
                case MINOTAUR_PORT4:
                    tmpPort = PORT_4;
                    break;
                default:
                    ROS_ERROR("SetModelSensors: Invalid sensor port: %d.", i);
                    return false;
            }
            sensorCommunicator.getSensorController().addSensor(tmpPort);
        }
        
    } catch(std::exception const &e) {
        ROS_ERROR("Failed to add sensors to SensorController : %s.", e.what());
        return false;
    }
    
    return true;
}

bool startThreads()
{
    int ret;
    
    ROS_INFO("Starting RobotThread...");
    
    ret = pthread_create(&robot_thread, NULL, robotThread, NULL);
    if(ret) {
        ROS_ERROR("Failed to create thread: %d", ret);
        return false;
    }
    
    ROS_INFO("Starting SensorThread...");
    
    ret = pthread_create(&sensor_thread, NULL, sensorThread, NULL);
    if(ret) {
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
        
        tmpSamplingMsec = getSamplingInterval();
        
        stepRobotController();
        
        end = ros::Time::now();
        sleepSec = MSEC_TO_SEC(tmpSamplingMsec) - (end - begin).toSec();
        
        if(sleepSec > 0)
            ros::Duration(sleepSec).sleep();
    }
    
    ROS_INFO("RobotThread terminated.");
    pthread_barrier_wait(&stopBarrier);
    ros::shutdown();
}

void stepRobotController()
{
    RAIILock lock(robotCommunicator.mutex());
    try {
        robotCommunicator.getRobotController().step(tmpSamplingMsec);
        //TODO is this the right place to send infos, maybe bad for performance
        //better to do it asynch?
        robotCommunicator.publish();
    }  catch(const nxtcon::USBException &ue) {
        ROS_ERROR("RobotThread: %s.", ue.what());
        shutdown();
    } catch(const std::exception &e) {
        ROS_WARN("RobotThread: %s.", e.what());
    } catch (...) {
        ROS_WARN("RobotThread: Caught unknown error.");
    }
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
        tmpSamplingMsec = getSamplingInterval();
        
        publishSensorData();
        
        end = ros::Time::now();
        sleepSec = MSEC_TO_SEC(tmpSamplingMsec) - (end - begin).toSec();
        
        if(sleepSec > 0)
            ros::Duration(sleepSec).sleep();
    }
    
    ROS_INFO("SensorThread terminated.");
    pthread_barrier_wait(&stopBarrier);
}

void publishSensorData()
{
    RAIILock lock(sensorCommunicator.mutex());
    try {
        sensorCommunicator.publish();
    } catch(nxtcon::USBException const &ue) {
        ROS_ERROR("SensorThread: %s.", ue.what());
        shutdown();
    } catch(std::exception const &e) {
        ROS_WARN("SensorThread: %s.", e.what());
    } catch(...) {
        ROS_WARN("SensorThread: Caught unknown error.");
    }
}

void joinThreads()
{
    pthread_join(robot_thread, NULL);
    pthread_join(sensor_thread, NULL);
}

int getSamplingInterval()
{
    RAIILock lock(&intervalMutex);
    return samplingMSec;
}
void setSamplingInterval(const int p_intervalMsec)
{
    RAIILock lock(&intervalMutex);
    samplingMSec = p_intervalMsec;
}

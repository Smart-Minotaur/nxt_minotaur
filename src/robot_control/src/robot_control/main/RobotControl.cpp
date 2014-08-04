#include <ros/ros.h>
#include <signal.h>
#include <exception>
#include <string>
#include "nxt_control/Brick.hpp"
#include "nxt_control/NxtOpcodes.hpp"
#include "nxt_control/NxtExceptions.hpp"
#include "robot_control/RobotThread.hpp"
#include "robot_control/SensorThread.hpp"
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

static nxtcon::Brick brick;
static minotaur::RobotCommunicator robotCommunicator;
static minotaur::SensorCommunicator sensorCommunicator;
static minotaur::RobotThread robotThread(robotCommunicator);
static minotaur::SensorThread sensorThread(sensorCommunicator);

/* Function prototypes */
bool init(ros::NodeHandle &p_handle, tf::TransformBroadcaster *p_broadcaster);
bool loadCurrentModel();
void startThreads();
void joinThreads();

static void signalHandler(int sig)
{
    ROS_INFO("Signal: Stop threads.");
    sensorThread.stop();
    robotThread.stop();
    joinThreads();
    ROS_INFO("Signal: Shutdown ROS.");
    ros::shutdown();
}

static void setSignals()
{
    signal(SIGINT, signalHandler);
    signal(SIGQUIT, signalHandler);
    signal(SIGTERM, signalHandler);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, NODE_NAME);
    ros::NodeHandle n;
    tf::TransformBroadcaster broadcaster;
    int ret;
    
    setSignals();
    
    if(!init(n, &broadcaster)) {
        ros::shutdown();
        return 1;
    }
    
    startThreads();
    ROS_INFO("%s-System ready for input.", NODE_NAME);
    
    ros::spin();
    
    return 0;
}

bool init(ros::NodeHandle& p_handle, tf::TransformBroadcaster *p_broadcaster)
{
    if(!ros::master::check()) {
        ROS_ERROR("Roscore has to be started.");
        return false;
    }
    
    try {
        ROS_INFO("Initialize USBConnection to Brick.");
        brick.find();
    } catch (const std::exception &e) {
        ROS_ERROR("Failed to initialize Brick: %s.", e.what());
        return false;
    }
    
    try {       
        robotCommunicator.setTransformBroadcaster(p_broadcaster);
        robotCommunicator.init(p_handle, &brick);
        sensorCommunicator.init(p_handle, &brick);
    } catch(const std::exception &e) {
        ROS_ERROR("Failed to initialize Communicator: %s.", e.what());
        return false;
    }
    
    if(!loadCurrentModel())
        return false;
    
    return true;
}

bool loadCurrentModel()
{   
    minotaur::RobotSettings robotSettings;
    std::vector<minotaur::SensorSetting> sensorSettings;
    
    ROS_INFO("Load current model.");
    try {
        robotSettings.loadCurrentFromParamServer();
        sensorSettings = minotaur::loadCurrentSensorSettings();
    } catch (const std::exception &e) {
        ROS_ERROR("Failed to load current model: %s.", e.what());
        return false;
    } catch (...) {
        ROS_ERROR("Failed to load current model: Catched unknown error.");
        return false;
    }
    
    ROS_INFO("Found model \"%s\". Parameter:", robotSettings.modelName.c_str());
    ROS_INFO("--Kp=%.4f; Ki=%.4f; Kd=%.4f", robotSettings.pidParameter.Kp, robotSettings.pidParameter.Ki, robotSettings.pidParameter.Kd);
    ROS_INFO("--wheelRadius=%.4f; wheelTrack=%.4f; interval=%dms", robotSettings.wheelRadius, robotSettings.wheelTrack, robotSettings.samplingInterval);
    for(int i = 0; i < sensorSettings.size(); ++i)
        ROS_INFO("--Sensor %d: id=%d; direction=%.2f; x=%.2f; y=%.2f", i, sensorSettings[i].id, sensorSettings[i].direction, sensorSettings[i].x, sensorSettings[i].y);
    robotCommunicator.applySettings(robotSettings);
    sensorCommunicator.applySettings(sensorSettings);
    
    return true;
}

void startThreads()
{
    ROS_INFO("Starting RobotThread.");
    robotThread.start();
    
    ROS_INFO("Starting SensorThread.");
    sensorThread.start();
}

void joinThreads()
{
    ROS_INFO("Joining threads.");
    sensorThread.join();
    robotThread.join();
}

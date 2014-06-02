#include <ros/ros.h>
#include <vector>
#include <exception>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <pthread.h>
#include <signal.h>
#include "minotaur_pc/RobotPosition.hpp"
#include "minotaur_pc/BugZeroAlgorithm.hpp"
#include "minotaur_pc/PLock.hpp"
#include "robot_control_beagle/UltrasonicData.h"
#include "robot_control_beagle/Utils.hpp"

#define ALGORITHM_FREQUENCY 10
#define NODE_NAME "buggyMinotaur"

volatile bool run;

ros::Subscriber odomSub;
ros::Subscriber measureSensorSub;
ros::Publisher targetPosPub;

std::vector<int> sensorMapping(3);
minotaur::BugZeroAlgorithm bugAlgorithm;
minotaur::RobotPosition currentPosition;

pthread_t bugThread;
pthread_mutex_t bugMutex;
pthread_mutex_t posMutex;

struct sigaction sa;

void setSignalAction();
void sighandler(int sig);
bool initBugzero();
bool initCommunication(ros::NodeHandle &p_handle);
void startThread();
void *bugThreadFunction(void *arg);
void bugLoop();
minotaur::RobotPosition execBugAlgorithm();
void processMeasureSensorMsg(const robot_control_beagle::UltrasonicData& p_msg);
void processOdometryMsg(const nav_msgs::Odometry& p_msgs);
void setTargetPosition(const minotaur::RobotPosition& p_position);

int main(int argc, char **argv)
{
    void *ret;
    ros::init(argc, argv, NODE_NAME);
    ros::NodeHandle handle;
    
    setSignalAction();
    
    if(!initBugzero())
        return -1;
    if(!initCommunication(handle))
        return -1;
    
    startThread();
    ros::spin();
    pthread_join(bugThread, &ret);
    
    return 0;
}

void setSignalAction()
{
    memset(&sa, 0, sizeof(struct sigaction));
    sa.sa_handler = sighandler;
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGQUIT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);
}

void sighandler(int sig)
{
    run = false;
}

bool initBugzero()
{
    ROS_INFO("Initializing BugZeroAlgorithm...");
    
    if(!ros::master::check()) {
        ROS_ERROR("Roscore has to be started.");
        return false;
    }
    
    std::string model;
    if(!ros::param::get(PARAM_CURRENT_MODEL(),model))
    {
        ROS_ERROR("Could not get CurrentModel from param-server.");
        return false;
    }
    
    int i = 0;
    while(ros::param::has(PARAM_SENSOR(model, i)))
    {
        float direction;
        if(!ros::param::get(PARAM_SENSOR_DIRECTION(model,i), direction))
        {
            ROS_ERROR("Sensor %d does not have a direction.", i);
            return false;
        }
        
        if(direction < 0)
            sensorMapping.push_back(LEFT_SENSOR);
        else if(direction > 0)
            sensorMapping.push_back(RIGHT_SENSOR);
        else
            sensorMapping.push_back(FRONT_SENSOR);
        ++i;
    }
    
    pthread_mutex_init(&bugMutex, NULL);
    pthread_mutex_init(&posMutex, NULL);
    
    return true;
}

bool initCommunication(ros::NodeHandle &p_handle)
{
    ROS_INFO("Initializing Communication...");
    if(!ros::master::check()) {
        ROS_ERROR("Roscore has to be started.");
        return false;
    }

    try {
        ROS_INFO("Subscribing on topic \"%s\"...", ROS_ODOM_TOPIC);
        odomSub = p_handle.subscribe(ROS_ODOM_TOPIC, 100, processOdometryMsg); 
        ROS_INFO("Subscribing on topic \"%s\"...", NXT_ULTRA_SENSOR_TOPIC);
        measureSensorSub = p_handle.subscribe(NXT_ULTRA_SENSOR_TOPIC, 100, processMeasureSensorMsg); 
        
        ROS_INFO("Publishing on topic \"%s\"...", ROS_SIMPLE_GOAL);
        targetPosPub = p_handle.advertise<geometry_msgs::PoseStamped>(ROS_SIMPLE_GOAL, 100);
    } catch(std::exception &e) {
        ROS_ERROR("InitCommunication: %s.", e.what());
        return false;
    }

    return true;
}

void startThread()
{
    run = true;
    pthread_create(&bugThread, NULL, bugThreadFunction, NULL);
}


void *bugThreadFunction(void *arg)
{
    ROS_INFO("BugAlgorithm started.");
    
    try
    {
        bugLoop();
    } catch (std::exception &e) {
        ROS_ERROR("BugAlgorithm: %s.", e.what());
    }
    
    ROS_INFO("BugAlgorithm terminated.");
    ros::shutdown();
}

void bugLoop()
{
    ros::Rate rate(ALGORITHM_FREQUENCY);
    minotaur::RobotPosition nextPosition;
    while(run)
    {
        nextPosition = execBugAlgorithm();
        setTargetPosition(nextPosition);
        rate.sleep();
    }
}

minotaur::RobotPosition execBugAlgorithm()
{
    minotaur::PLock lock1(&bugMutex);
    minotaur::PLock lock2(&posMutex);
    return bugAlgorithm.getNextPosition(currentPosition);
}

void processMeasureSensorMsg(const robot_control_beagle::UltrasonicData& p_msg)
{
    minotaur::PLock lock(&bugMutex);
    bugAlgorithm.setSensorValue(sensorMapping[p_msg.sensorID], p_msg.distance);
}

void processOdometryMsg(const nav_msgs::Odometry& p_msgs)
{
    minotaur::PLock lock(&posMutex);
    currentPosition.point.x = p_msgs.pose.pose.position.x;
    currentPosition.point.y = p_msgs.pose.pose.position.y;
    currentPosition.theta = p_msgs.pose.pose.position.z;
}

void setTargetPosition(const minotaur::RobotPosition& p_position)
{
        geometry_msgs::PoseStamped msg;
        
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = MINOTAUR_BASE_FRAME;
        
        msg.pose.position.x = p_position.point.x;
        msg.pose.position.y = p_position.point.y;
        msg.pose.position.z = p_position.theta;
        
        msg.pose.orientation = tf::createQuaternionMsgFromYaw(p_position.theta);
        
        targetPosPub.publish(msg);
}
#include <ros/ros.h>
#include <vector>
#include <exception>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <pthread.h>
#include "minotaur_pc/RobotPosition.hpp"
#include "robot_control_beagle/UltrasonicData.h"
#include "robot_control_beagle/Utils.hpp"

#define ALGORITHM_FREQUENCY 10
#define NODE_NAME "buggyMinotaur"

volatile bool run;

ros::Subscriber odomSub;
ros::Subscriber measureSensorSub;
ros::Publisher targetPosPub;

std::vector<int> sensorMapping(3);

pthread_t bugThread;

void initBugzero();
bool initCommunication(ros::NodeHandle &p_handle);
void startThread();
void *bugThreadFunction(void *arg);
void bugLoop();
void processMeasureSensorMsg(const robot_control_beagle::UltrasonicData& p_msg);
void processOdometryMsg(const nav_msgs::Odometry& p_msgs);
void setTargetPosition(const minotaur::RobotPosition& p_position);

int main(int argc, char **argv)
{
    ros::init(argc, argv, NODE_NAME);
    ros::NodeHandle handle;
    
    if(!initCommunication(handle))
        return -1;
    
    
    ros::spin();
    return 0;
}

void initBugzero()
{
    
}

bool initCommunication(ros::NodeHandle &p_handle)
{
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
    while(run)
    {
        //TODO
        rate.sleep();
    }
}

void processMeasureSensorMsg(const robot_control_beagle::UltrasonicData& p_msg)
{
    //TODO
}

void processOdometryMsg(const nav_msgs::Odometry& p_msgs)
{
    //TODO
}

void setTargetPosition(const minotaur::RobotPosition& p_position)
{
        geometry_msgs::PoseStamped msg;
        
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = MINOTAUR_BASE_FRAME;
        
        msg.pose.position.x = p_position.point.x;
        msg.pose.position.y = p_position.point.y;
        msg.pose.position.z = 0;
        
        msg.pose.orientation = tf::createQuaternionMsgFromYaw(p_position.theta);
        
        targetPosPub.publish(msg);
}
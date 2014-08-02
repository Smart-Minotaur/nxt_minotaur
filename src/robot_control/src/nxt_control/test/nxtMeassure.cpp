#include <ros/ros.h>
#include <signal.h>
#include "minotaur_common/GetUltrasonic.h"
#include "minotaur_common/AddUltrasonic.h"
#include "minotaur_common/MinotaurTopics.hpp"

#define MEASSURE_FREQ 60

int ultrasonicID = -1;
ros::ServiceClient ultrasonicClient, addUltrasonicClient;
bool run = true;

void initClients(ros::NodeHandle &p_handle);
void setSignalAction();
void sighandler(int sig);
void addUltrasonic();
void measureLoop();

int main(int argc, char **argv)
{
    ros::init(argc, argv, "nxtUltrasonicSensor");
    ros::NodeHandle n;
    
    setSignalAction();
    
    initClients(n);
    addUltrasonic();
    
    measureLoop();
    
    ros::shutdown();
    
    return 0;
}

void setSignalAction()
{  
    signal(SIGINT, sighandler);
    signal(SIGQUIT, sighandler);
    signal(SIGTERM, sighandler);
}

void sighandler(int sig)
{
    ROS_INFO("===========================");
    ROS_INFO("Meassure cancelled");
    run = false;
}

void initClients(ros::NodeHandle &p_handle)
{
    ROS_INFO("Initializing service connections...");
    
    // to connect to certain services its name is needed
    // the templyte types are the message types used to communicate
    ultrasonicClient = p_handle.serviceClient<minotaur_common::GetUltrasonic>(MINOTAUR_GET_ULTRASONIC_SRV);
    addUltrasonicClient = p_handle.serviceClient<minotaur_common::AddUltrasonic>(MINOTAUR_ADD_ULTRASONIC_SRV);
}

void addUltrasonic()
{
    minotaur_common::AddUltrasonic srv;
    srv.request.port = SENSOR_PORT1;
    
    ROS_INFO("Adding Ultrasonicsensor...");
    if(addUltrasonicClient.call(srv))
    {
        ultrasonicID = srv.response.sensorID;
        ROS_INFO("Done. Got ID: %d.", ultrasonicID);
    }
    else
    {
        ROS_ERROR("Could not add UltrasonicSensor on port %d.", SENSOR_PORT1);
    }
}

void measureLoop()
{
    minotaur_common::GetUltrasonic srv;
    srv.request.sensorID = ultrasonicID;
    ros::Rate loopRate(MEASSURE_FREQ);
    unsigned int distance;
    
    if(ultrasonicID < 0)
        return;
    
    while(run)
    {
        ultrasonicClient.call(srv);
        distance = srv.response.distance;
        ROS_INFO("Distance: %d", distance);
        loopRate.sleep();
    }
}

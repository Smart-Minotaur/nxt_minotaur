/*
 * Author: Fabian Meyer 
 */

#include <ros/ros.h>
#include "nxt_beagle/nxtUltrasonic.h"
#include "nxt_beagle/nxtAddUltrasonic.h"
#include "nxt_beagle/Config.hpp"

#define TEST_COUNT 100

int ultrasonicID = -1;
ros::ServiceClient ultrasonicClient, addUltrasonicClient;
double ultrasonicLatency = -1.0;

void initClients(ros::NodeHandle &p_handle);
void addUltrasonic();
void testUltrasonicLatency();
void printResults();

int main(int argc, char **argv)
{
    ros::init(argc, argv, "nxtLatency");
    ros::NodeHandle n;
    
    initClients(n);
    addUltrasonic();
    
    ROS_INFO("Test begins");
    ROS_INFO("=====================");
    
    testUltrasonicLatency();
    printResults();
    
    ROS_INFO("=====================");
    ROS_INFO("Finished.");
    
    return 0;
}

void initClients(ros::NodeHandle &p_handle)
{
    ROS_INFO("Initializing service connections...");
    
    // to connect to certain services its name is needed
    // the templyte types are the message types used to communicate
    ultrasonicClient = p_handle.serviceClient<nxt_beagle::nxtUltrasonic>(NXT_GET_ULTRASONIC_SRV);
    addUltrasonicClient = p_handle.serviceClient<nxt_beagle::nxtAddUltrasonic>(NXT_ADD_ULTRASONIC_SRV);
}

void addUltrasonic()
{
    nxt_beagle::nxtAddUltrasonic srv;
    srv.request.port = NXT_PORT1;
    
    ROS_INFO("Adding Ultrasonicsensor...");
    if(addUltrasonicClient.call(srv))
    {
        ultrasonicID = srv.response.sensorID;
        ROS_INFO("Done. Got ID: %d.", ultrasonicID);
    }
    else
    {
        ROS_ERROR("Could not add UltrasonicSensor on port %d.", NXT_PORT1);
    }
}

void testUltrasonicLatency()
{
    if(ultrasonicID < 0)
        return;
    
    ros::Time begin, end;
    nxt_beagle::nxtUltrasonic srv;
    srv.request.sensorID = ultrasonicID;
    
    ROS_INFO("Testing Ultrasonic Latency...");
    
    begin = ros::Time::now();
    
    for(int i = 0; i < TEST_COUNT; i++)
    {
        if(!ultrasonicClient.call(srv))
        {
            ROS_ERROR("Failed to get Ultrasonic data. Try %d.", i);
            return;
        }
    }
    
    end = ros::Time::now();
    ultrasonicLatency = ((end - begin).toSec() / TEST_COUNT) * 1000;
}

void printResults()
{
    if(ultrasonicLatency >= 0)
        ROS_INFO("Ultrasonic latency is: %f ms", ultrasonicLatency);
}
#include <ros/ros.h>
#include "nxt_beagle/nxtTicks.h"
#include "nxt_beagle/nxtUltrasonic.h"
#include "nxt_beagle/nxtAddUltrasonic.h"

#define GET_TICKS_SRV "get_ticks"
#define GET_UTLTRASONIC_SRV "get_ultrasonic"
#define ADD_UTLTRASONIC_SRV "add_ultrasonic"
#define ULTRASONIC_PORT 1
#define TEST_COUNT 100

int ultrasonicID = -1;
ros::ServiceClient tickClient, ultrasonicClient, addUltrasonicClient;
double tickLatency = -1.0;
double ultrasonicLatency = -1.0;

void initClients(ros::NodeHandle &p_handle);
void addUltrasonic();
void testUltrasonicLatency();
void testTicksLatency();
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
    testTicksLatency();
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
    tickClient = p_handle.serviceClient<nxt_beagle::nxtTicks>(GET_TICKS_SRV);
    ultrasonicClient = p_handle.serviceClient<nxt_beagle::nxtUltrasonic>(GET_UTLTRASONIC_SRV);
    addUltrasonicClient = p_handle.serviceClient<nxt_beagle::nxtAddUltrasonic>(ADD_UTLTRASONIC_SRV);
}

void addUltrasonic()
{
    nxt_beagle::nxtAddUltrasonic srv;
    srv.request.port = ULTRASONIC_PORT;
    
    ROS_INFO("Adding Ultrasonicsensor...");
    if(addUltrasonicClient.call(srv))
    {
        ultrasonicID = srv.response.sensorID;
        ROS_INFO("Done. Got ID: %d.", ultrasonicID);
    }
    else
    {
        ROS_ERROR("Could not add UltrasonicSensor on port %d.", ULTRASONIC_PORT);
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

void testTicksLatency()
{
    nxt_beagle::nxtTicks srv;
    ros::Time begin, end;
    
    ROS_INFO("Testing Ticks Latency...");
    
    begin = ros::Time::now();
    
    for(int i = 0; i < TEST_COUNT; i++)
    {
        if(!tickClient.call(srv))
        {
            ROS_ERROR("Failed to get Ticks. Try %d.", i);
            return;
        }
    }
    
    end = ros::Time::now();
    tickLatency = ((end - begin).toSec() / TEST_COUNT) * 1000;
}

void printResults()
{
    if(ultrasonicLatency >= 0)
        ROS_INFO("Ultrasonic latency is: %f ms", ultrasonicLatency);
    if(tickLatency >= 0)
        ROS_INFO("Ticks latency is: %f ms", tickLatency);
}
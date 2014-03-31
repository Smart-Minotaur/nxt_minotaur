#include <ros/ros.h>
#include "nxt_minotaur/nxtUltrasonic.h"
#include "nxt_minotaur/nxtAddUltrasonic.h"
#include <signal.h>

#define GET_UTLTRASONIC_SRV "get_ultrasonic"
#define ADD_UTLTRASONIC_SRV "add_ultrasonic"
#define ULTRASONIC_PORT 1
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
    ultrasonicClient = p_handle.serviceClient<nxt_minotaur::nxtUltrasonic>(GET_UTLTRASONIC_SRV);
    addUltrasonicClient = p_handle.serviceClient<nxt_minotaur::nxtAddUltrasonic>(ADD_UTLTRASONIC_SRV);
}

void addUltrasonic()
{
    nxt_minotaur::nxtAddUltrasonic srv;
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

void measureLoop()
{
    nxt_minotaur::nxtUltrasonic srv;
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
        ros::spinOnce();
        loopRate.sleep();
    }
}
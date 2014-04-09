#include <ros/ros.h>
#include <pthread.h>
#include <signal.h>
#include "nxt_beagle/nxtUltrasonic.h"
#include "nxt_beagle/nxtAddUltrasonic.h"
#include "nxt_beagle/Config.hpp"

#define MEASSURE_FREQ 5

int ultrasonicID_sensor1 = -1;
int ultrasonicID_sensor2 = -1;
int ultrasonicID_sensor3 = -1;
bool run = true;
ros::ServiceClient ultrasonicClient, addUltrasonicClient;
pthread_t thread;

void initClients(ros::NodeHandle &p_handle);
void setSignalAction();
void sighandler(int sig);
void addUltrasonics();
void* sensorThread(void *arg);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "nxtUltrasonicSensor");
    ros::NodeHandle n;
    int ret;
    
    initClients(n);
    addUltrasonics();
    
    ROS_INFO("Starting SensorThread...");
    ret = pthread_create(&thread, NULL, sensorThread, NULL);
    if(ret)
    {
        ROS_ERROR("Failed to create sensorThread: %d", ret);
        return -1;
    }
    
    pthread_join(thread, NULL);
    
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
    ultrasonicClient = p_handle.serviceClient<nxt_beagle::nxtUltrasonic>(NXT_GET_ULTRASONIC_SRV);
    addUltrasonicClient = p_handle.serviceClient<nxt_beagle::nxtAddUltrasonic>(NXT_ADD_ULTRASONIC_SRV);
}

void addUltrasonics()
{
    nxt_beagle::nxtAddUltrasonic srv1, srv2, srv3;
    srv1.request.port = NXT_PORT1;
    srv2.request.port = NXT_PORT2;
    srv3.request.port = NXT_PORT3;
    
    ROS_INFO("Adding Ultrasonicsensor...");
    if(addUltrasonicClient.call(srv1) && addUltrasonicClient.call(srv2) && addUltrasonicClient.call(srv3))
    {
        ultrasonicID_sensor1 = srv1.response.sensorID;
	ultrasonicID_sensor2 = srv2.response.sensorID;
	ultrasonicID_sensor3 = srv3.response.sensorID;
        ROS_INFO("Success! Got SensorIDS: %d, %d, %d.", ultrasonicID_sensor1, ultrasonicID_sensor2, ultrasonicID_sensor3);
    }
    else
    {
        ROS_ERROR("Could not add UltrasonicSensors!");
    }
}

void* sensorThread(void *arg)
{
  nxt_beagle::nxtUltrasonic srv1, srv2, srv3;
  srv1.request.sensorID = ultrasonicID_sensor1;
  srv2.request.sensorID = ultrasonicID_sensor2;
  srv3.request.sensorID = ultrasonicID_sensor3;
  ros::Rate loopRate(MEASSURE_FREQ);
  unsigned int distance1, distance2, distance3;
  
  if(ultrasonicID_sensor1 < 0 || ultrasonicID_sensor2 < 0 || ultrasonicID_sensor3 < 0)
        return NULL;
  
  while(run)
    {
        ultrasonicClient.call(srv1);
	ultrasonicClient.call(srv2);
	ultrasonicClient.call(srv3);
        distance1 = srv1.response.distance;
	distance2 = srv2.response.distance;
	distance3 = srv3.response.distance;
        ROS_INFO("Distances:  %d || %d  || %d ", distance1, distance2, distance3);
        ros::spinOnce();
        loopRate.sleep();
    }
}

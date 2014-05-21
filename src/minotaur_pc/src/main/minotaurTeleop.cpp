#include <ros/ros.h>
#include <pthread.h>
#include <tf/transform_broadcaster.h>
#include <signal.h>
#include <termios.h>
#include <vector>
#include "nxt_beagle/Config.hpp"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Pose.h"
#include "minotaur_pc/MapCreator.hpp"
#include "nxt_beagle/UltraSensor.h"
#include "minotaur_pc/BlockingQueue.hpp"
#include "minotaur_pc/RobotPosition.hpp"

#define MAX_LINEAR_VEL 0.2
#define MAX_ANGULAR_VEL 1.5

#define KEYCODE_R 0x43
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_Q 0x71

volatile bool run;

ros::Subscriber odometrySub;
ros::Subscriber ultraSensorSub;
ros::Publisher robotVelocityPublisher;

minotaur::MapCreator mapCreator;
minotaur::BlockingQueue<nxt_beagle::UltraSensor> queue;
std::vector<int> sensor;

pthread_mutex_t positionMutex;
pthread_t eventThread;
pthread_t mapThread;

minotaur::RobotPosition currentPosition;

struct termios oldtio, currtio;
struct sigaction sa;

void setSignalAction();
void sighandler(int sig);
bool initCommunication(ros::NodeHandle &p_handle);
bool initMapCreator();
void processOdomMsg(const nav_msgs::Odometry p_msg);
void processUltraSensorMsg(const nxt_beagle::UltraSensor p_msg);
void initKeyHandling();
void startThreads();
void joinThreads();
void *eventLoop(void * p_arg);
void *mapLoop(void * arg);

int main(int argc, char** argv)
{   
    ros::init(argc, argv, "minotaurTeleop");
    ros::NodeHandle handle;
    
    setSignalAction();
    if(!initCommunication(handle))
        return -1;
    if(!initMapCreator())
        return -2;
    initKeyHandling();
    
    startThreads();
    while(run)
        ros::spinOnce();
    joinThreads();
    
    ROS_INFO("Saving Map...");
    mapCreator.createTextFile("/home/fabian/teleop_grid");
    
    ros::shutdown();
    
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
    nxt_beagle::UltraSensor poisonPill;
    poisonPill.sensorID = 4;
    run = false;
    queue.enqueue(poisonPill);
}

bool initCommunication(ros::NodeHandle &p_handle)
{
    if(!ros::master::check())
    {
        ROS_ERROR("roscore has to be started.");
        return false;
    }
    
    ROS_INFO("Subscribing to topic \"%s\"...", ROS_ODOM_TOPIC);
    odometrySub = p_handle.subscribe(ROS_ODOM_TOPIC, 50, processOdomMsg);
    
    ROS_INFO("Subscribing to topic \"%s\"...", NXT_ULTRA_SENSOR_TOPIC);
    ultraSensorSub = p_handle.subscribe(NXT_ULTRA_SENSOR_TOPIC, 50, processUltraSensorMsg);
    
    ROS_INFO("Publishing on topic \"%s\"...", ROS_VEL_TOPIC);
    robotVelocityPublisher = p_handle.advertise<geometry_msgs::Twist>(ROS_VEL_TOPIC, 50);
    
    currentPosition.point.x = 0;
    currentPosition.point.y = 0;
    currentPosition.theta = 0;
    
    return true;
}

bool initMapCreator()
{
    std::string model;
    if(!ros::param::get(PARAM_CURRENT_MODEL(), model))
    {
        ROS_ERROR("Unable to get current Model.");
        return false;
    }
    
    ROS_INFO("Extracted \"%s\" as current Model.", model.c_str());
    
    int i = 0;
    float direction, dx, dy;
    sensor.clear();
    while(true)
    {
        if(!ros::param::has(PARAM_SENSOR(model, i)))
            break;
        ROS_INFO("Adding Sensor %d...", i);
        
        if(!ros::param::get(PARAM_SENSOR_DIRECTION(model, i), direction))
        {
            ROS_ERROR("Sensor %d does not have direction parameter.", i);
            return false;
        }
        
        if(!ros::param::get(PARAM_SENSOR_DX(model, i), dx))
        {
            ROS_ERROR("Sensor %d does not have dx parameter.", i);
            return false;
        }
        
        if(!ros::param::get(PARAM_SENSOR_DY(model, i), dy))
        {
            ROS_ERROR("Sensor %d does not have dy parameter.", i);
            return false;
        }
        
        if(direction > 0)
            sensor.push_back(MAP_CREATOR_LEFT_SENSOR);
        else if(direction < 0)
            sensor.push_back(MAP_CREATOR_RIGHT_SENSOR);
        else
            sensor.push_back(MAP_CREATOR_FRONT_SENSOR);
        
        mapCreator.setSensorDistances(sensor[i], dx, dy);
        ++i;
    }
    
    return true;
}

void processOdomMsg(const nav_msgs::Odometry p_msg)
{
    pthread_mutex_lock(&positionMutex);
    currentPosition.convert(p_msg.pose);
    pthread_mutex_unlock(&positionMutex);
}

void processUltraSensorMsg(const nxt_beagle::UltraSensor p_msg)
{
    queue.enqueue(p_msg);
}

void initKeyHandling()
{
    //save for restore
    tcgetattr(0, &oldtio);
    
    // put the console in raw mode
    tcgetattr(0, &currtio);
    currtio.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(0, TCSANOW, &currtio);

    puts("Reading from keyboard");
    puts("---------------------------");
    puts("Use arrow keys to move Minotaur.");
}

void startThreads()
{
    run = true;
    pthread_mutex_init(&positionMutex, NULL);
    pthread_create(&eventThread, NULL, eventLoop, NULL);
    //pthread_create(&mapThread, NULL, mapLoop, NULL);
}

void joinThreads()
{
    void *ret;
    pthread_join(eventThread, &ret);
    //pthread_join(mapThread, &ret);
}

void *eventLoop(void * p_arg)
{
    char keycode;
    ros::Rate loop_rate(60);
    geometry_msgs::Twist msg;
    minotaur::RobotPosition currentPositionCopy;
    
    msg.linear.x = 0;
    msg.linear.y = 0;
    msg.linear.z = 0;
    
    msg.angular.x = 0;
    msg.angular.y = 0;
    msg.angular.z = 0;
    
    while(run)
    {
        if(read(0, &keycode, 1) < 0)
        {
            perror("read():");
            break;
        }
        
        pthread_mutex_lock(&positionMutex);
        currentPositionCopy = currentPosition;
        pthread_mutex_unlock(&positionMutex);

        switch(keycode)
        {
            case KEYCODE_L:
                msg.linear.x = 0;
                msg.linear.y = 0;
                msg.angular.z = MAX_ANGULAR_VEL;
                robotVelocityPublisher.publish(msg);
                break;
            case KEYCODE_R:
                msg.linear.x = 0;
                msg.linear.y = 0;
                msg.angular.z = -MAX_ANGULAR_VEL;
                robotVelocityPublisher.publish(msg);
                break;
            case KEYCODE_U:
                msg.linear.x = cos(currentPositionCopy.theta) * MAX_LINEAR_VEL;
                msg.linear.y = sin(currentPositionCopy.theta) * MAX_LINEAR_VEL;
                msg.angular.z = 0;
                robotVelocityPublisher.publish(msg);
                break;
            case KEYCODE_D:
                msg.linear.x = cos(currentPositionCopy.theta) * -MAX_LINEAR_VEL;
                msg.linear.y = sin(currentPositionCopy.theta) * -MAX_LINEAR_VEL;
                msg.angular.z = 0;
                robotVelocityPublisher.publish(msg);
                break;
            case KEYCODE_Q:
                msg.linear.x = 0;
                msg.linear.y = 0;
                msg.angular.z = 0;
                robotVelocityPublisher.publish(msg);
                break;
        }
            
        loop_rate.sleep();
    }
    
    tcsetattr(0, TCSANOW, &oldtio);
    ROS_INFO("Eventloop terminated.");
    
    return NULL;
}

void *mapLoop(void * arg)
{
    nxt_beagle::UltraSensor sensorMsg;
    
    while(run)
    {
        sensorMsg = queue.dequeue();
        
        if(sensorMsg.sensorID < 4)
        {
            pthread_mutex_lock(&positionMutex);
            mapCreator.setPosition(currentPosition);
            pthread_mutex_unlock(&positionMutex);
            
            mapCreator.step(sensor[sensorMsg.sensorID], sensorMsg.distance);
        }
    }
    
    ROS_INFO("MapLoop terminated.");
    
    return NULL;
}
#include <ros/ros.h>
#include <signal.h>
#include "minotaur_map/SensorThread.hpp"
#include "minotaur_map/NavigationThread.hpp"
#include "minotaur_map/MinotaurCommunicator.hpp"
#include "minotaur_map/RobotOdometry.hpp"
#include "minotaur_map/SensorMeasurement.hpp"
#include "minotaur_common/BlockingQueue.hpp"

#define NODE_NAME "Minotaur"

minotaur::MinotaurCommunicator communicator;
minotaur::NavigationThread navigationThread;
minotaur::SensorThread sensorThread;
minotaur::RobotOdometry odometry;
minotaur::BlockingQueue<minotaur::SensorMeasurement> queue;

bool init(ros::NodeHandle& p_handle);
void setSignals();
void signalHandler(int sig);
void start();
void join();

int main(int argc, char **argv)
{
    ros::init(argc, argv, NODE_NAME);
    ros::NodeHandle handle;
    
    if(!init(handle))
        return 1;
    
    setSignals();
    start();
    
    ros::spin();
    
    return 0;
}

bool init(ros::NodeHandle& p_handle)
{
    if(!ros::master::check()) {
        ROS_ERROR("Roscore has to be started.");
        return false;
    }
    
    try {
        communicator.init(p_handle, &odometry, &queue);
        sensorThread.init(&odometry, &queue);
        navigationThread.init(&communicator, &odometry);
    } catch (std::exception const &e) {
        ROS_ERROR("Failed to initialize components: %s.", e.what());
        return false;
    }
    
    return true;
}

void setSignals()
{
    signal(SIGINT, signalHandler);
    signal(SIGQUIT, signalHandler);
    signal(SIGTERM, signalHandler);
}

void signalHandler(int sig)
{
    sensorThread.stop();
    navigationThread.stop();
    join();
    ros::shutdown();
}

void start()
{
    navigationThread.start();
    sensorThread.start();
}

void join()
{
    navigationThread.join();
    sensorThread.join();
}

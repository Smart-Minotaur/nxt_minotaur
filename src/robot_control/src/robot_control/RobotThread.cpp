#include <ros/ros.h>
#include "robot_control/RobotThread.hpp"    
#include "minotaur_common/RAIILock.hpp"
#include "minotaur_common/Math.hpp"

#define DEF_SAMPLING_INTERVAL 25

namespace minotaur
{
    
    RobotThread::RobotThread(RobotCommunicator &p_robotCommunicator)
    : keepRunning(false), robotCommunicator(p_robotCommunicator), samplingIntervalMsec(DEF_SAMPLING_INTERVAL)
    {
        pthread_mutex_init(&intervalMutex, NULL);
    }
    
    RobotThread::~RobotThread()
    {
        pthread_mutex_destroy(&intervalMutex);
    }
    
    void RobotThread::onStart()
    {
        keepRunning = true;
    }
    
    void RobotThread::onStop()
    {
        keepRunning = false;
    }
    
    void RobotThread::run()
    {
        ROS_INFO("RobotThread: started.");
        
        while(keepRunning) {
            int samplingIntervalTMP = getSamplingInterval();
            ros::Rate loopRate(msecToHz(samplingIntervalTMP));
            
            stepRobotController(samplingIntervalTMP);
            
            loopRate.sleep();
        }
        
        ROS_INFO("RobotThread: terminated.");
    }
    
    void RobotThread::stepRobotController(const int p_samplingInterval)
    {
        try {
            robotCommunicator.stepController(p_samplingInterval);
            robotCommunicator.publish();
        }  catch(const nxt::NXTException &e) {
            ROS_ERROR("RobotThread: %s.", e.what());
            stop();
        } catch(const std::exception &e) {
            ROS_WARN("RobotThread: %s.", e.what());
        } catch (...) {
            ROS_WARN("RobotThread: Caught unknown error.");
        }
    }
    
    void RobotThread::setSamplingInterval(const int p_samplingIntervalMsec)
    {
        RAIILock lock(&intervalMutex);
        samplingIntervalMsec = p_samplingIntervalMsec;
    }
    
    int RobotThread::getSamplingInterval()
    {
        RAIILock lock(&intervalMutex);
        return samplingIntervalMsec;
    }
}

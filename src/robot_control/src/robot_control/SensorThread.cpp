#include <ros/ros.h>
#include "robot_control/SensorThread.hpp"
#include "nxt_control/NxtExceptions.hpp"
#include "minotaur_common/Math.hpp"
#include "minotaur_common/RAIILock.hpp"

namespace minotaur
{
    SensorThread::SensorThread(SensorCommunicator &p_sensorCommunicator)
    : keepRunning(false), sensorCommunicator(p_sensorCommunicator)
    {
        pthread_mutex_init(&intervalMutex, NULL);
    }
    
    SensorThread::~SensorThread()
    {
        pthread_mutex_destroy(&intervalMutex);
    }
        
    void SensorThread::onStart()
    {
        keepRunning = true;
    }
    
    void SensorThread::onStop()
    {
        keepRunning = false;
    }
    
    void SensorThread::run()
    {
        ROS_INFO("SensorThread: started.");
        
        while(keepRunning) {
            ros::Rate loopRate(msecToHz(getSamplingInterval()));
            publishSensorData();
            loopRate.sleep();
        }
        
        ROS_INFO("SensorThread: terminated.");
    }
    
    void SensorThread::publishSensorData()
    {
        try {
            sensorCommunicator.publish();
        } catch(const nxtcon::USBException &ue) {
            ROS_ERROR("SensorThread: %s.", ue.what());
        } catch(const std::exception &e) {
            ROS_WARN("SensorThread: %s.", e.what());
        } catch(...) {
            ROS_WARN("SensorThread: Caught unknown error.");
        }
    }
    
    void SensorThread::setSamplingInterval(const int p_samplingIntervalMsec)
    {
        RAIILock lock(&intervalMutex);
        samplingIntervalMsec = p_samplingIntervalMsec;
    }
    
    int SensorThread::getSamplingInterval()
    {
        RAIILock lock(&intervalMutex);
        return samplingIntervalMsec;
    }
}

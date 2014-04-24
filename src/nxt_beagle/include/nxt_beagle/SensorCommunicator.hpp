#ifndef NXT_BEAGLE_SENSOR_COMMUNICATOR_HPP_
#define NXT_BEAGLE_SENSOR_COMMUNICATOR_HPP_

#include <ros/ros.h>
#include <pthread.h>
#include "nxt_beagle/SensorController.hpp"
#include "nxt_beagle/nxtAddUltrasonic.h"
#include "nxt_beagle/nxtUltrasonic.h"
#include "nxt_beagle/ClearSensor.h"

namespace minotaur
{
    class SensorCommunicator
    {
    private:
        SensorController sensorController;
        
        ros::Subscriber clearSensorSub;

        ros::ServiceServer addUltraSonicSrv;
        ros::ServiceServer getUltraSonicSrv;
        
        ros::Publisher sensorDataPub;
        
        pthread_mutex_t sensorMutex;
        
        /* callbacks */
        void processClearSensorMsg(const nxt_beagle::ClearSensor &p_msg);
        bool processGetUltrasonicRqt(nxt_beagle::nxtUltrasonic::Request  &req,
                                     nxt_beagle::nxtUltrasonic::Response &res);
        bool processAddUltrasonicRqt(nxt_beagle::nxtAddUltrasonic::Request  &req,
                                     nxt_beagle::nxtAddUltrasonic::Response &res);
    public:
        SensorCommunicator() { }
        virtual ~SensorCommunicator() { }
        
        void init(ros::NodeHandle &p_handle, nxtcon::Brick *p_brick);
        
        void publish();
        
        void lock() { pthread_mutex_lock(&sensorMutex); }
        void unlock() { pthread_mutex_unlock(&sensorMutex); }
        
        SensorController& getSensorController();
    };
}

#endif
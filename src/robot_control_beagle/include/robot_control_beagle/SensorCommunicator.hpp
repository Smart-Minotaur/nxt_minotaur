#ifndef ROBOT_CONTROL_SENSOR_COMMUNICATOR_HPP_
#define ROBOT_CONTROL_SENSOR_COMMUNICATOR_HPP_

#include <ros/ros.h>
#include <pthread.h>
#include "robot_control_beagle/SensorController.hpp"
#include "robot_control_beagle/AddUltrasonic.h"
#include "robot_control_beagle/GetUltrasonic.h"
#include "robot_control_beagle/ClearSensor.h"

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
        void processClearSensorMsg(const robot_control_beagle::ClearSensor &p_msg);
        bool processGetUltrasonicRqt(robot_control_beagle::GetUltrasonic::Request  &req,
                                     robot_control_beagle::GetUltrasonic::Response &res);
        bool processAddUltrasonicRqt(robot_control_beagle::AddUltrasonic::Request  &req,
                                     robot_control_beagle::AddUltrasonic::Response &res);
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
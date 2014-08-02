#ifndef ROBOT_CONTROL_SENSOR_COMMUNICATOR_HPP_
#define ROBOT_CONTROL_SENSOR_COMMUNICATOR_HPP_

#include <ros/ros.h>
#include <pthread.h>
#include "robot_control/SensorController.hpp"
#include "minotaur_common/AddUltrasonic.h"
#include "minotaur_common/GetUltrasonic.h"
#include "minotaur_common/ClearSensor.h"
#include "minotaur_common/SensorSettings.hpp"

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
        void processClearSensorMsg(const minotaur_common::ClearSensor &p_msg);
        bool processGetUltrasonicRqt(minotaur_common::GetUltrasonic::Request  &req,
                                     minotaur_common::GetUltrasonic::Response &res);
        bool processAddUltrasonicRqt(minotaur_common::AddUltrasonic::Request  &req,
                                     minotaur_common::AddUltrasonic::Response &res);
    public:
        SensorCommunicator();
        ~SensorCommunicator();
        
        void init(ros::NodeHandle &p_handle, nxtcon::Brick *p_brick);
        void publish();
        void applySettings(const std::vector<SensorSetting> p_settings);
    };
}

#endif

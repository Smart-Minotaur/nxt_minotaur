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
    /**
     * \brief The SensorCommunicator reads sensor-values and publishes them to the subscribers
     * 
     */
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
	/**
	 * Clear all added sensors
	 * @param p_msg empty message - this param is used as a trigger for ROS
	 */
        void processClearSensorMsg(const minotaur_common::ClearSensor &p_msg);
	/**
	 * Request to get ultrasonic data at a certain port
	 * @param req Request-message: contains the port-id
	 * @param res Response-message
	 */
        bool processGetUltrasonicRqt(minotaur_common::GetUltrasonic::Request  &req,
                                     minotaur_common::GetUltrasonic::Response &res);
	/**
	 * Request to add an ultrasonic sensor
	 * @param req The request-message: contains the requested port-id
	 * @param res The response-message
	 * @retval processAddUltrasonicRqt true if sensor added successfully, else false
	 */
        bool processAddUltrasonicRqt(minotaur_common::AddUltrasonic::Request  &req,
                                     minotaur_common::AddUltrasonic::Response &res);
    public:
        SensorCommunicator(nxt::Brick *p_brick);
        ~SensorCommunicator();
        
        void init(ros::NodeHandle &p_handle);
	/**
	 * Reads the values from sensors and publishes them in UltrasonicData messages
	 */
        void publish();
        void applySettings(const std::vector<SensorSetting> p_settings);
    };
}

#endif

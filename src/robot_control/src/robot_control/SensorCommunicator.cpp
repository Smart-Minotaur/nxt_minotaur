#include <exception>
#include <nxt/NXTControl.hpp>
#include "robot_control/SensorCommunicator.hpp"
#include "minotaur_common/UltrasonicData.h"
#include "minotaur_common/MinotaurTopics.hpp"
#include "minotaur_common/RAIILock.hpp"

#define ROS_MSG_QUEUE_LENGTH 20

namespace minotaur
{
    SensorCommunicator::SensorCommunicator(nxt::Brick *p_brick)
	:sensorController(p_brick)
    {
        pthread_mutex_init(&sensorMutex, NULL);
    }
    
    SensorCommunicator::~SensorCommunicator()
    {
        pthread_mutex_destroy(&sensorMutex);
    }
    
    void SensorCommunicator::init(ros::NodeHandle &p_handle)
    {
        RAIILock lock(&sensorMutex);
        
        ROS_INFO("-- Setting up SensorController.");
        
        ROS_INFO("-- Publishing on topic \"%s\".", MINOTAUR_MEASURE_SENSOR_TOPIC);
        sensorDataPub = p_handle.advertise<minotaur_common::UltrasonicData>(MINOTAUR_MEASURE_SENSOR_TOPIC, ROS_MSG_QUEUE_LENGTH); 
        
        ROS_INFO("-- Subscribing to topic \"%s\".", MINOTAUR_CLEAR_SENSOR_TOPIC);
        clearSensorSub = p_handle.subscribe(MINOTAUR_CLEAR_SENSOR_TOPIC, ROS_MSG_QUEUE_LENGTH, &SensorCommunicator::processClearSensorMsg, this);
        
        ROS_INFO("-- Offering service \"%s\".", MINOTAUR_GET_ULTRASONIC_SRV);
        getUltraSonicSrv = p_handle.advertiseService(MINOTAUR_GET_ULTRASONIC_SRV, &SensorCommunicator::processGetUltrasonicRqt, this);
        ROS_INFO("--Offering service \"%s\".", MINOTAUR_ADD_ULTRASONIC_SRV);
        addUltraSonicSrv = p_handle.advertiseService(MINOTAUR_ADD_ULTRASONIC_SRV, &SensorCommunicator::processAddUltrasonicRqt, this);
    }
    
    void SensorCommunicator::publish()
    {
        RAIILock lock(&sensorMutex);
        
        minotaur_common::UltrasonicData msg;
        ros::Rate sendRate(50);
        for(int i = 0; i < sensorController.count(); ++i) {
            try {
                msg.sensorID = i;
                msg.distance = sensorController.getDistance(i);
                sensorDataPub.publish(msg);
            } catch (const nxt::TimeoutException &e) {
                ROS_WARN("SensorCommunicator: %s.", e.what());
            } catch (const std::exception &e) {
                ROS_ERROR("SensorCommunicator: %s.", e.what());
            } catch (...) {
                ROS_ERROR("SensorCommunicator: Caught unknown error (publish()).");
            }
            sendRate.sleep();
        }
    }
    
    void SensorCommunicator::applySettings(const std::vector<SensorSetting> p_settings)
    {
        RAIILock lock(&sensorMutex);
        
        sensorController.clearSensors();
        for(int i = 0; i < p_settings.size(); ++i) {
            uint8_t port;
            switch(i) {
                case SENSOR_PORT1:
                    port = PORT_1;
                    break;
                case SENSOR_PORT2:
                    port = PORT_2;
                    break;
                case SENSOR_PORT3:
                    port = PORT_3;
                    break;
                case SENSOR_PORT4:
                    port = PORT_4;
                    break;
                default:
                    std::stringstream ss;
                    ss << "Got invalid sensor port:" << i;
                    throw std::logic_error(ss.str());
                    break;
            }
            sensorController.addSensor(port);
        }
    }
    
    /* callbacks */
    void SensorCommunicator::processClearSensorMsg(const minotaur_common::ClearSensor &p_msg)
    {
        RAIILock lock(&sensorMutex);
        sensorController.clearSensors();
    }

    bool SensorCommunicator::processGetUltrasonicRqt(minotaur_common::GetUltrasonic::Request  &req,
                                                     minotaur_common::GetUltrasonic::Response &res)
    {
        RAIILock lock(&sensorMutex);
        bool result = true;
        try {
            res.distance = sensorController.getDistance(req.sensorID);
        } catch(const nxt::TimeoutException &te) {
            ROS_WARN("GetUltrasonicData: %s.", te.what());
            result = false;
        } catch(const std::exception &e) {
            ROS_ERROR("GetUltrasonicData: %s.", e.what());
            result = false;
        } catch (...) {
            ROS_ERROR("SensorCommunicator: Caught unknown error (processGetUltrasonicRqt()).");
            result = false;
        }
        return result;
    }

    bool SensorCommunicator::processAddUltrasonicRqt(minotaur_common::AddUltrasonic::Request  &req,
                                                     minotaur_common::AddUltrasonic::Response &res)
    {
        bool result = true;
        uint8_t tmpPort;
        switch(req.port)
        {
            case SENSOR_PORT1:
                tmpPort = PORT_1;
                break;
            case SENSOR_PORT2:
                tmpPort = PORT_2;
                break;
            case SENSOR_PORT3:
                tmpPort = PORT_3;
                break;
            case SENSOR_PORT4:
                tmpPort = PORT_4;
                break;
            default:
                ROS_ERROR("AddUltrasonicSensor: Invalid sensor port: %d.", req.port);
                return false;
        }
        
        RAIILock lock(&sensorMutex);
        try {
            res.sensorID = sensorController.addSensor(tmpPort);
        } catch(std::exception const &e) {
            ROS_ERROR("AddUltrasonicSensor: %s.", e.what());
            result =  false;
        } catch (...) {
            ROS_ERROR("AddUltrasonicSensor: Caught unknown error (processAddUltrasonicRqt()).");
            result = false;
        }
        return result;
    }
}

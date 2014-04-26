#include "nxt_beagle/SensorCommunicator.hpp"
#include "nxt_beagle/Config.hpp"
#include "nxt_beagle/UltraSensor.h"
#include "nxt_control/NxtOpcodes.hpp"
#include "nxt_control/NxtExceptions.hpp"

namespace minotaur
{
    void SensorCommunicator::init(ros::NodeHandle &p_handle, nxtcon::Brick *p_brick)
    {
        ROS_INFO("Publishing on topic \"%s\"...", NXT_ULTRA_SENSOR_TOPIC);
        sensorDataPub = p_handle.advertise<nxt_beagle::UltraSensor>(NXT_ULTRA_SENSOR_TOPIC, 1000); 
        
        ROS_INFO("Subscribing to topic \"%s\"...", NXT_CLEAR_SENSOR_TOPIC);
        clearSensorSub = p_handle.subscribe(NXT_CLEAR_SENSOR_TOPIC, 1000, &SensorCommunicator::processClearSensorMsg, this);
        
        ROS_INFO("Offering service \"%s\"...", NXT_GET_ULTRASONIC_SRV);
        getUltraSonicSrv = p_handle.advertiseService(NXT_GET_ULTRASONIC_SRV, &SensorCommunicator::processGetUltrasonicRqt, this);
        ROS_INFO("Offering service \"%s\"...", NXT_ADD_ULTRASONIC_SRV);
        addUltraSonicSrv = p_handle.advertiseService(NXT_ADD_ULTRASONIC_SRV, &SensorCommunicator::processAddUltrasonicRqt, this);
        
        ROS_INFO("Setting up SensorController...");
        sensorController.setBrick(p_brick);
    }
    
    void SensorCommunicator::publish()
    {
        nxt_beagle::UltraSensor msg;
        for(int i = 0; i < sensorController.count(); ++i)
        {
            msg.sensorID = i;
            msg.distance = sensorController.getDistance(i);
            sensorDataPub.publish(msg);
        }
    }
    
    SensorController& SensorCommunicator::getSensorController()
    {
        return sensorController;
    }
    
    /* callbacks */
    void SensorCommunicator::processClearSensorMsg(const nxt_beagle::ClearSensor &p_msg)
    {
        lock();
        //always try catch between mutex lock / unlock to prevent deadlock
        try
        {
            sensorController.clearSensors();
        }
        catch(std::exception const &e)
        {
            ROS_ERROR("ClearSensors: %s.", e.what());
        }
        unlock();
    }

    bool SensorCommunicator::processGetUltrasonicRqt(nxt_beagle::nxtUltrasonic::Request  &req,
                                nxt_beagle::nxtUltrasonic::Response &res)
    {
        bool result = true;
        
        lock();
        //always try catch between mutex lock / unlock to prevent deadlock
        try
        {
            res.distance = sensorController.getDistance(req.sensorID);
        }
        catch(nxtcon::NXTTimeoutException const &te)
        {
            ROS_WARN("GetUltrasonicData: %s.", te.what());
        }
        catch(std::exception const &e)
        {
            ROS_ERROR("GetUltrasonicData: %s.", e.what());
            result = false;
        }
        unlock();
        return result;
    }

    bool SensorCommunicator::processAddUltrasonicRqt(nxt_beagle::nxtAddUltrasonic::Request  &req,
                                 nxt_beagle::nxtAddUltrasonic::Response &res)
    {
        bool result = true;
        uint8_t tmpPort;
        switch(req.port)
        {
            case NXT_PORT1:
                tmpPort = PORT_1;
                break;
            case NXT_PORT2:
                tmpPort = PORT_2;
                break;
            case NXT_PORT3:
                tmpPort = PORT_3;
                break;
            case NXT_PORT4:
                tmpPort = PORT_4;
                break;
            default:
                ROS_ERROR("AddUltrasonicSensor: Invalid sensor port: %d.", req.port);
                return false;
        }
        
        lock();
        //always try catch between mutex lock / unlock to prevent deadlock
        try
        {
            res.sensorID = sensorController.addSensor(tmpPort);
        }
        catch(std::exception const &e)
        {
            ROS_ERROR("AddUltrasonicSensor: %s.", e.what());
            result =  false;
        }
        unlock();
        return result;
    }
}
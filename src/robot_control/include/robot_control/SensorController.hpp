#ifndef ROBOT_CONTROL_SENSOR_CONTROLLER_HPP_
#define ROBOT_CONTROL_SENSOR_CONTROLLER_HPP_

#include <vector>
#include "nxt_control/Brick.hpp"
#include "nxt_control/UltrasonicSensor.hpp"

namespace minotaur
{
    /**
     * \brief Gets distance-information from the sensors
     * 
     * This class is used for the SensorCommunicator
     */
    class SensorController
    {
    private:
        nxtcon::Brick *brick;
        std::vector<nxtcon::UltrasonicSensor> sensors;
        
    public:
        SensorController()
        { }
        
        virtual ~SensorController() { }
        
        void setBrick(nxtcon::Brick *p_brick);
        
	/**
	 * get the last measured distance from a specific sensor
	 * @param p_id id of the sensor
	 * @retval getDistance measured distance
	 */
        uint8_t getDistance(const uint8_t p_id);
        uint8_t addSensor(const uint8_t p_port);
        void clearSensors();
        
        int count();
    };
}

#endif
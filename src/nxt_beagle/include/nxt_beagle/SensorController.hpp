#ifndef NXT_BEAGLE_SENSOR_CONTROLLER_HPP_
#define NXT_BEAGLE_SENSOR_CONTROLLER_HPP_

#include <vector>
#include "nxt_control/Brick.hpp"
#include "nxt_control/UltrasonicSensor.hpp"

namespace minotaur
{
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
        
        uint8_t getDistance(const uint8_t p_id);
        uint8_t addSensor(const uint8_t p_port);
        void clearSensors();
        
        int sensorCount() const;
    };
}

#endif
#ifndef NXT_CONTROL_ULTRASONICSENSOR_HPP_
#define NXT_CONTROL_ULTRASONICSENSOR_HPP_

#include "nxt_control/Brick.hpp"
#include "nxt_control/NxtContainer.hpp"

namespace nxtcon
{
    class UltrasonicSensor
    {
    private:
        Brick *brick;
        uint8_t port;
        SensorData sensor;
        
    public:
        UltrasonicSensor(Brick *p_brick, const uint8_t p_port);
        
        virtual ~UltrasonicSensor() { }
   
        const SensorData& getSensorData();
    };
}

#endif
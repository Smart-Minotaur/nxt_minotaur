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
        
        uint8_t getStatus(const int p_timeoutMS);
    public:
        UltrasonicSensor(Brick *p_brick, const uint8_t p_port);
        
        virtual ~UltrasonicSensor() { }
   
        uint8_t getDistance(const int p_timeoutMS = 0);
    };
}

#endif
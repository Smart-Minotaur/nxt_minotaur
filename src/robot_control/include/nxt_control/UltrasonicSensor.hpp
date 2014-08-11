#ifndef NXT_CONTROL_ULTRASONICSENSOR_HPP_
#define NXT_CONTROL_ULTRASONICSENSOR_HPP_

#include <sys/time.h>
#include <unistd.h>
#include "nxt_control/Brick.hpp"
#include "nxt_control/NxtContainer.hpp"

namespace nxtcon
{
    /**
     * \brief This class represents an ultrasonic-sensor
     * 
     */
    class UltrasonicSensor
    {
    private:
        Brick *brick;
        uint8_t port;
        
        struct timeval lastPoll;
        useconds_t pollDelayUsec;
        
        uint8_t getStatus();
        void keepPollInterval();
    public:
        UltrasonicSensor(Brick *p_brick, const uint8_t p_port);
        
        virtual ~UltrasonicSensor() { }
   
        uint8_t getDistance();
    };
}

#endif
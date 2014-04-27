#ifndef NXT_CONTROL_MOTOR_HPP_
#define NXT_CONTROL_MOTOR_HPP_

#include "nxt_control/Brick.hpp"
#include "nxt_control/NxtContainer.hpp"

namespace nxtcon
{
    
    class Motor
    {
    private:
        Brick *brick;
        uint8_t port;
        TachoData tacho;
        
    public:
        Motor()
        :brick(NULL), port(0), tacho() { }
        Motor(Brick *p_brick, const uint8_t p_port)
        :brick(p_brick),port(p_port), tacho() { }
        
        virtual ~Motor() { }
        
        void setBrick(Brick *p_brick);
        void setPort(const uint8_t p_port);
   
        void brake();
        void setPower(const int8_t p_power);
        void resetTacho();
        const TachoData& getTachoData();
        
    };
}

#endif
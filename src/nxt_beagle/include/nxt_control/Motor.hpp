#ifndef NXT_CONTROL_MOTOR_HPP_
#define NXT_CONTROL_MOTOR_HPP_

#include "nxt_control/Brick.hpp"
#include "nxt_control/NxtTelegram.hpp"

namespace minotaur
{
    
    class TachoData
    {
    public:
        /* Count since last motor reset */
        int32_t tachoCount;
        /* Count since last programmed movement (last command)*/
        int32_t blockTachoCount;
        /* Count since last reset of rotation sensor of this motor*/
        int32_t rotationCount;
        
        
        TachoData(const int32_t p_tachoCount, const int32_t p_blocktachoCount, const int32_t p_rotationCount)
        :tachoCount(p_tachoCount), blockTachoCount(p_blocktachoCount), rotationCount(p_rotationCount) { }
        virtual ~TachoData { }
        
        void set(const int32_t p_tachoCount, const int32_t p_blocktachoCount, const int32_t p_rotationCount)
        {tachoCount = p_tachoCount; blockTachoCount = p_blocktachoCount; rotationCount = p_rotationCount;s}
    }
    
    class Motor
    {
    private:
        Brick *brick;
        uint8_t port;
        TachoData tacho;
        
    public:
        Motor(Brick *p_brick, const uint8_t p_port)
        :brick(p_brick),port(p_port), tacho() { }
        
        virtual ~Motor() { }
   
        void setMotorPower(const int8_t p_power);
        void resetTacho();
        TachoData& getTachoData();
        
    };
}

#endif
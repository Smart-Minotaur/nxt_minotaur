#ifndef NXT_CONTROL_NXT_TELEGRAM_HPP_
#define NXT_CONTROL_NXT_TELEGRAM_HPP_

#include "nxt_control/Telegram.hpp"
#include "nxt_control/NxtOpcodes.hpp"

namespace minotaur
{
    void create_setMotor(Telegram *p_telegram, const uint8_t p_port, const uint8_t p_power);
    void create_brakeMotor(Telegram *p_telegram, const uint8_t p_port);
    void create_getOutputState(Telegram *p_telegram, const uint8_t p_port);
    void create_resetTacho(Telegram *p_telegram, const uint8_t p_port);
    void create_getInputValues(Telegram *p_telegram, const uint8_t p_port);
    
}

#endif
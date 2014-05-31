#ifndef NXT_CONTROL_NXT_TELEGRAM_HPP_
#define NXT_CONTROL_NXT_TELEGRAM_HPP_

#include "nxt_control/Telegram.hpp"
#include "nxt_control/NxtOpcodes.hpp"
#include "nxt_control/NxtContainer.hpp"

namespace nxtcon
{
    void create_setMotor(Telegram *p_telegram, const uint8_t p_port, const uint8_t p_power);
    void create_brakeMotor(Telegram *p_telegram, const uint8_t p_port);
    void create_getOutputState(Telegram *p_telegram, const uint8_t p_port);
    void create_resetTacho(Telegram *p_telegram, const uint8_t p_port);
    void create_getInputValues(Telegram *p_telegram, const uint8_t p_port);
    void create_setInputMode(Telegram *p_telegram, const uint8_t p_port, const uint8_t p_type, const uint8_t p_mode);
    void create_resetInputScaledValue(Telegram *p_telegram, const uint8_t p_port);
    
    void create_setUltraSonicPingMode(Telegram *p_telegram, const uint8_t p_port);
    void create_measureUltraSonic(Telegram *p_telegram, const uint8_t p_port);
    void create_lsGetStatus(Telegram *p_telegram, const uint8_t p_port);
    void create_lsRead(Telegram *p_telegram, const uint8_t p_port);
    
    void decode_tachoOutputState(const Telegram &p_telegram, TachoData *p_tacho, const uint8_t p_port);
    void decode_ultraSonicSensorInputValues(const Telegram &p_telegram, SensorData *p_sensor, const uint8_t p_port);
    uint8_t decode_ultraSonicStatus(const Telegram &p_telegram, const uint8_t p_port);
    
    bool isLittleEndian();
}

#endif
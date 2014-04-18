#include "nxt_control/NxtTelegram.hpp"

namespace minotaur
{
    void create_setMotor(Telegram *p_telegram, const uint8_t p_port, const uint8_t p_power)
    {
        p_telegram->clear();
        //set length of message => LSB (0xC0 = 12; without length bytes)
        p_telegram->add(0xC0);
        p_telegram->add(0x00);
        //set command type
        p_telegram->add(DIRECT_CMD_NO_REPLY);
        //set command
        p_telegram->add(CMD_SET_OUTPUT_STATE);
        //set motor port
        p_telegram->add_uint8(p_port);
        //set motor power
        p_telegram->add_uint8(p_power);
        //set motor mode
        p_telegram->add(MOTOR_MODE_ON_BRAKE);
        //set regulation mode
        p_telegram->add(REGULATION_MODE_NONE);
        //set turn ratio
        p_telegram->add(0x00);
        //set run state
        p_telegram->add(RUN_STATE_RUNNING);
        //set tacholimit => LSB
        p_telegram->add(0x00);
        p_telegram->add(0x00);
        p_telegram->add(0x00);
        p_telegram->add(0x00);
    }
    
    void create_brakeMotor(Telegram *p_telegram, const uint8_t p_port)
    {
        p_telegram->clear();
        //set length of message => LSB (0xC0 = 12; without length bytes)
        p_telegram->add(0xC0);
        p_telegram->add(0x00);
        //set command type
        p_telegram->add(DIRECT_CMD_NO_REPLY);
        //set command
        p_telegram->add(CMD_SET_OUTPUT_STATE);
        //set motor port
        p_telegram->add_uint8(p_port);
        //set motor power
        p_telegram->add_uint8(0x00);
        //set motor mode
        p_telegram->add(MOTOR_MODE_ON_BRAKE);
        //set regulation mode
        p_telegram->add(REGULATION_MODE_NONE);
        //set turn ratio
        p_telegram->add(0x00);
        //set run state
        p_telegram->add(RUN_STATE_IDLE);
        //set tacholimit => LSB
        p_telegram->add(0x00);
        p_telegram->add(0x00);
        p_telegram->add(0x00);
        p_telegram->add(0x00);
    }
    
    void create_getOutputState(Telegram *p_telegram, const uint8_t p_port)
    {
        p_telegram->clear();
        //set length of message => LSB (0x30 = 3; without length bytes)
        p_telegram->add(0x30);
        p_telegram->add(0x00);
        //set command type
        p_telegram->add(DIRECT_CMD);
        //set command
        p_telegram->add(CMD_GET_OUTPUT_STATE);
        //set motor port
        p_telegram->add_uint8(p_port);
    }
    
    void create_resetTacho(Telegram *p_telegram, const uint8_t p_port)
    {
         p_telegram->clear();
        //set length of message => LSB (0x30 = 3; without length bytes)
        p_telegram->add(0x30);
        p_telegram->add(0x00);
        //set command type
        p_telegram->add(DIRECT_CMD_NO_REPLY);
        //set command
        p_telegram->add(CMD_RESET_MOTOR);
        //set motor port
        p_telegram->add_uint8(p_port);
    }
    
    void create_getInputValues(Telegram *p_telegram, const uint8_t p_port)
    {
        p_telegram->clear();
        //set length of message => LSB (0x30 = 3; without length bytes)
        p_telegram->add(0x30);
        p_telegram->add(0x00);
        //set command type
        p_telegram->add(DIRECT_CMD);
        //set command
        p_telegram->add(CMD_GET_INPUT_VALUES);
        //set sensor port
        p_telegram->add_uint8(p_port);
    }
}
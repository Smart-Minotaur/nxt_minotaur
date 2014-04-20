#include <sstream>
#include "nxt_control/NxtTelegram.hpp"
#include "nxt_control/NxtExceptions.hpp"

namespace nxtcon
{
    void create_setMotor(Telegram *p_telegram, const uint8_t p_port, const uint8_t p_power)
    {
        p_telegram->clear();
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
        //set command type
        p_telegram->add(DIRECT_CMD);
        //set command
        p_telegram->add(CMD_GET_INPUT_VALUES);
        //set sensor port
        p_telegram->add_uint8(p_port);
    }
    
    void decode_tachoOutputState(Telegram &p_telegram, TachoData *p_tacho, const uint8_t p_port)
    {
        unsigned char data[p_telegram.getLength()];
        p_telegram.getData(data);
        if(data[0] != REPLY_TELEGRAM)
            throw std::logic_error("Cannot decode non-reply-telegram.");
        if(data[1] != CMD_GET_OUTPUT_STATE)
            throw std::logic_error("Telegram is not of type getOutputState. Cannot decode tacho.");
        if(data[2] != 0x00)
            throw NXTCommunicationException("getOutputState returned error. Cannot decode tacho.", (int) data[2]);
        if(data[3]!= p_port)
        {
            std::stringstream ss;
            ss << "Cannot decode tacho. Wrong port. Expected: " << p_port << " Got: " << data[3];
            throw std::logic_error(ss.str());
        }
        
        unsigned char *ptr = (unsigned char*) &(p_tacho->tachoCount);
        //little-endian to big-endian
        ptr[0] = data[16];
        ptr[1] = data[15];
        ptr[2] = data[14];
        ptr[3] = data[13];
        
        ptr = (unsigned char*) &(p_tacho->blockTachoCount);
        //little-endian to big-endian
        ptr[0] = data[20];
        ptr[1] = data[19];
        ptr[2] = data[18];
        ptr[3] = data[17];
        
        ptr = (unsigned char*) &(p_tacho->rotationCount);
        //little-endian to big-endian
        ptr[0] = data[24];
        ptr[1] = data[23];
        ptr[2] = data[22];
        ptr[3] = data[21];
    }
    
    void decode_unltaSonicSensorInputValues(Telegram &p_telegram, SensorData *p_sensor, const uint8_t p_port)
    {
        unsigned char data[p_telegram.getLength()];
        p_telegram.getData(data);
        if(data[0] != REPLY_TELEGRAM)
            throw std::logic_error("Cannot decode non-reply-telegram.");
        if(data[1] != CMD_GET_INPUT_VALUES)
            throw std::logic_error("Telegram is not of type getInputValues. Cannot decode sensor.");
        if(data[2] != 0x00)
            throw NXTCommunicationException("getInputValues returned error. Cannot decode sensor.", (int) data[2]);
        if(data[3]!= p_port)
        {
            std::stringstream ss;
            ss << "Cannot decode sensor. Wrong port. Expected: " << p_port << " Got: " << data[3];
            throw std::logic_error(ss.str());
        }
        
        p_sensor->valid = data[4];
        p_sensor->calibrated = data[5];
        
        unsigned char *ptr = (unsigned char*) &(p_sensor->rawValue);
        //little-endian to big-endian
        ptr[0] = data[9];
        ptr[1] = data[8];
        
        ptr = (unsigned char*) &(p_sensor->normalizedValue);
        //little-endian to big-endian
        ptr[0] = data[11];
        ptr[1] = data[10];
        
        ptr = (unsigned char*) &(p_sensor->scaledValue);
        //little-endian to big-endian
        ptr[0] = data[13];
        ptr[1] = data[12];
        
        ptr = (unsigned char*) &(p_sensor->calibratedValue);
        //little-endian to big-endian
        ptr[0] = data[15];
        ptr[1] = data[14];
    }
}
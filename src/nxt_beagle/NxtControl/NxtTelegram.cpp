#include <sstream>
#include <stdexcept>
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
        p_telegram->add(RUN_STATE_RUNNING);
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
        //set relative to last position (true) or absolute (false)
        p_telegram->add(0x01);
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
    
    void create_setInputMode(Telegram *p_telegram, const uint8_t p_port, const uint8_t p_type, const uint8_t p_mode)
    {
        p_telegram->clear();
        //set command type
        p_telegram->add(DIRECT_CMD_NO_REPLY);
        //set command
        p_telegram->add(CMD_SET_INPUT_MODE);
        //set sensor port
        p_telegram->add_uint8(p_port);
        //set sensor type
        p_telegram->add_uint8(p_type);
        //set sensor mode
        p_telegram->add_uint8(p_mode);
    }
    
    void create_resetInputScaledValue(Telegram *p_telegram, const uint8_t p_port)
    {
        p_telegram->clear();
        //set command type
        p_telegram->add(DIRECT_CMD_NO_REPLY);
        //set command
        p_telegram->add(CMD_RESET_INPUT_SCALED_VALUE);
        //set sensor port
        p_telegram->add_uint8(p_port);
    }
    
    void create_setUltraSonicPingMode(Telegram *p_telegram, const uint8_t p_port)
    {
        p_telegram->clear();
        //set command type
        p_telegram->add(DIRECT_CMD_NO_REPLY);
        //set command
        p_telegram->add(CMD_LSWRITE);
        //set sensor port
        p_telegram->add_uint8(p_port);
        //set length of data to send
        p_telegram->add(0x03);
        //set length of data to receive
        p_telegram->add(0x00);
        //ultrasonic address for ls
        p_telegram->add(LS_ULTRASONIC_ADDRESS);
        //command set mode
        p_telegram->add(LS_SET_ULTRASONIC_MODE);
        //set mode to ping mode
        p_telegram->add(ULTRASONIC_MODE_SINGLESHOT);
    }
    
    void create_measureUltraSonic(Telegram *p_telegram, const uint8_t p_port)
    {
        p_telegram->clear();
        //set command type
        p_telegram->add(DIRECT_CMD_NO_REPLY);
        //set command
        p_telegram->add(CMD_LSWRITE);
        //set sensor port
        p_telegram->add_uint8(p_port);
        //set length of data to send
        p_telegram->add(0x02);
        //set length of data to receive
        p_telegram->add(0x01);
        //ultrasonic address for ls
        p_telegram->add(LS_ULTRASONIC_ADDRESS);
        //command set mode
        p_telegram->add(READ_ULTRASONIC_BYTE0);
    }
    
    void create_lsGetStatus(Telegram *p_telegram, const uint8_t p_port)
    {
        p_telegram->clear();
        //set command type
        p_telegram->add(DIRECT_CMD);
        //set command
        p_telegram->add(CMD_LSGETSTATUS);
        //set sensor port
        p_telegram->add_uint8(p_port);
    }
    
    void create_lsRead(Telegram *p_telegram, const uint8_t p_port)
    {
        p_telegram->clear();
        //set command type
        p_telegram->add(DIRECT_CMD);
        //set command
        p_telegram->add(CMD_LSREAD);
        //set sensor port
        p_telegram->add_uint8(p_port);
    }
    
    void decode_tachoOutputState(const Telegram &p_telegram, TachoData *p_tacho, const uint8_t p_port)
    {
        unsigned char data[p_telegram.getLength()];
        p_telegram.getData(data);
        if(data[0] != REPLY_TELEGRAM)
            throw std::logic_error("Cannot decode non-reply-telegram");
        if(data[1] != CMD_GET_OUTPUT_STATE)
            throw std::logic_error("Telegram is not of type getOutputState. Cannot decode tacho");
        if(data[2] != 0x00)
            throw NXTCommunicationException("getOutputState returned error. Cannot decode tacho", (int) data[2]);
        if(data[3]!= p_port)
        {
            std::stringstream ss;
            ss << "Cannot decode tacho. Wrong port. Expected: " << p_port << " Got: " << data[3];
            throw std::logic_error(ss.str());
        }
        
        unsigned char *ptr;
        if(isLittleEndian())
        {
            //nxt is little-endian, this machine also
            ptr = (unsigned char*) &(p_tacho->tachoCount);
            ptr[0] = data[13];
            ptr[1] = data[14];
            ptr[2] = data[15];
            ptr[3] = data[16];
            
            ptr = (unsigned char*) &(p_tacho->blockTachoCount);
            ptr[0] = data[17];
            ptr[1] = data[18];
            ptr[2] = data[19];
            ptr[3] = data[20];
            
            ptr = (unsigned char*) &(p_tacho->rotationCount);
            ptr[0] = data[21];
            ptr[1] = data[22];
            ptr[2] = data[23];
            ptr[3] = data[24];
        }
        else
        {
            //nxt is little-endian, this machine is big-endian
            ptr = (unsigned char*) &(p_tacho->tachoCount);
            ptr[0] = data[16];
            ptr[1] = data[15];
            ptr[2] = data[14];
            ptr[3] = data[13];
            
            ptr = (unsigned char*) &(p_tacho->blockTachoCount);
            ptr[0] = data[20];
            ptr[1] = data[19];
            ptr[2] = data[18];
            ptr[3] = data[17];
            
            ptr = (unsigned char*) &(p_tacho->rotationCount);
            ptr[0] = data[24];
            ptr[1] = data[23];
            ptr[2] = data[22];
            ptr[3] = data[21];
        }
    }
    
    void decode_ultraSonicSensorInputValues(const Telegram &p_telegram, SensorData *p_sensor, const uint8_t p_port)
    {
        unsigned char data[p_telegram.getLength()];
        p_telegram.getData(data);
        if(data[0] != REPLY_TELEGRAM)
            throw std::logic_error("Cannot decode non-reply-telegram");
        if(data[1] != CMD_GET_INPUT_VALUES)
            throw std::logic_error("Telegram is not of type getInputValues. Cannot decode sensor");
        if(data[2] != 0x00)
            throw NXTCommunicationException("getInputValues returned error. Cannot decode sensor", (int) data[2]);
        if(data[3]!= p_port)
        {
            std::stringstream ss;
            ss << "Cannot decode sensor. Wrong port. Expected: " << p_port << " Got: " << data[3];
            throw std::logic_error(ss.str());
        }
        
        p_sensor->valid = data[4];
        p_sensor->calibrated = data[5];
        
        unsigned char *ptr;
        if(isLittleEndian())
        {
            //nxt is little-endian, this machine also
            ptr = (unsigned char*) &(p_sensor->rawValue);
            ptr[0] = data[8];
            ptr[1] = data[9];
            
            ptr = (unsigned char*) &(p_sensor->normalizedValue);
            ptr[0] = data[10];
            ptr[1] = data[11];
            
            ptr = (unsigned char*) &(p_sensor->scaledValue);
            ptr[0] = data[12];
            ptr[1] = data[13];
            
            ptr = (unsigned char*) &(p_sensor->calibratedValue);
            ptr[0] = data[14];
            ptr[1] = data[15];
        }
        else
        {
            //nxt is little-endian, this machine is big-endian
            ptr = (unsigned char*) &(p_sensor->rawValue);
            ptr[0] = data[9];
            ptr[1] = data[8];
            
            ptr = (unsigned char*) &(p_sensor->normalizedValue);
            ptr[0] = data[11];
            ptr[1] = data[10];
            
            ptr = (unsigned char*) &(p_sensor->scaledValue);
            ptr[0] = data[13];
            ptr[1] = data[12];
            
            ptr = (unsigned char*) &(p_sensor->calibratedValue);
            ptr[0] = data[15];
            ptr[1] = data[14];
        }
    }
    
    bool isLittleEndian()
    {
        int num = 1;
        return (*((char *) &num) == 1);
    }
}
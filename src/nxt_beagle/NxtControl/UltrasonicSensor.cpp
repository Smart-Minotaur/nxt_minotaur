#include "nxt_control/UltrasonicSensor.hpp"
#include "nxt_control/NxtTelegram.hpp"
#include "nxt_control/NxtExceptions.hpp"
#include <stdexcept>
#include <sys/time.h>
#include <ros/ros.h>
#include <unistd.h>

#define LS_STATUS_RESPONSE_LENGTH 4
#define MAX_LS_READ_RESPONSE 20

namespace nxtcon
{
    int msdiff(struct timeval * p_begin, struct timeval *p_end);
    
    UltrasonicSensor::UltrasonicSensor(Brick *p_brick, const uint8_t p_port)
    :brick(p_brick),port(p_port)
    {
        if(!brick->isConnected())
            throw NXTException("Cannot create UltrasonicSensor. Brick is not connected or has not been searched.");
        
        Telegram telegram;
        create_setInputMode(&telegram, port, SENSOR_TYPE_LOWSPEED_9V, SENSOR_MODE_RAW);
        brick->send(telegram);
        create_setUltraSonicPingMode(&telegram, port);
        brick->send(telegram);
    }
    
    uint8_t UltrasonicSensor::getDistance(const int p_timeoutMS)
    {
        Telegram telegram;
        unsigned char data [MAX_LS_READ_RESPONSE];
        
        create_measureUltraSonic(&telegram, port);
        brick->send(telegram);
        if(!getStatus(p_timeoutMS))
            throw NXTTimeoutException("Ultrasonic measurement timed out.");
        
        create_lsRead(&telegram, port);
        telegram = brick->sendWithResponse(telegram);
        telegram.getData(data);
        
        return data[4];
    }
    
    uint8_t UltrasonicSensor::getStatus(const int p_timeoutMS)
    {
        Telegram send, receive;
        unsigned char data[LS_STATUS_RESPONSE_LENGTH];
        struct timeval begin, end;
        int ms = 0;
        
        create_lsGetStatus(&send, port);
        do
        {
            if(p_timeoutMS > 0)
                gettimeofday(&begin, NULL);
            
            receive = brick->sendWithResponse(send);
            receive.getData(data);
            
            if(p_timeoutMS > 0)
            {
                gettimeofday(&end, NULL);
                ms += msdiff(&begin, &end);
            }
            
        } while (data[2] == 0 && (p_timeoutMS <= 0 || ms < p_timeoutMS));
        
        return data[2];
    }
    
    int msdiff(struct timeval * p_begin, struct timeval *p_end)
    {
        int result = (p_end->tv_sec - p_begin->tv_sec) * 1000;
        result += (p_end->tv_usec - p_begin->tv_usec) / 1000;
        return result;
    }
}
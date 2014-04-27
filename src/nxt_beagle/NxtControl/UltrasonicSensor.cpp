#include "nxt_control/UltrasonicSensor.hpp"
#include "nxt_control/NxtTelegram.hpp"
#include "nxt_control/NxtExceptions.hpp"
#include <stdexcept>
#include <ros/ros.h>

#define LS_STATUS_RESPONSE_LENGTH 4
#define MAX_LS_READ_RESPONSE 20
#define DEF_DELAY_USEC 10000
#define STATUS_TIMEOUT_COUNT 30

namespace nxtcon
{
    int msdiff(struct timeval * p_begin, struct timeval *p_end);
    useconds_t usdiff(struct timeval * p_begin, struct timeval *p_end);
    
    UltrasonicSensor::UltrasonicSensor(Brick *p_brick, const uint8_t p_port)
    :brick(p_brick),port(p_port)
    {
        if(!brick->isConnected())
            throw NXTException("Cannot create UltrasonicSensor. Brick is not connected or has not been searched");
        
        Telegram telegram;
        create_setInputMode(&telegram, port, SENSOR_TYPE_LOWSPEED_9V, SENSOR_MODE_RAW);
        brick->send(telegram);
        /*create_setUltraSonicPingMode(&telegram, port);
        brick->send(telegram);*/
        
        pollDelayUsec = DEF_DELAY_USEC;
        gettimeofday(&lastPoll,NULL);
    }
    
    uint8_t UltrasonicSensor::getDistance()
    {
        Telegram telegram;
        unsigned char data [MAX_LS_READ_RESPONSE];
        uint8_t ret;
        
        keepPollInterval();
        
        create_measureUltraSonic(&telegram, port);
        brick->send(telegram);
        try
        {
            ret = getStatus();
        }
        catch(std::exception const &e)
        {
            create_lsRead(&telegram, port);
            telegram = brick->sendWithResponse(telegram);
            throw e;
        }
        
        create_lsRead(&telegram, port);
        telegram = brick->sendWithResponse(telegram);
        
        if(!ret)
            throw NXTTimeoutException("Ultrasonic measurement timed out");
        
        telegram.getData(data);
        
        return data[4];
    }
    
     void UltrasonicSensor::keepPollInterval()
     {
         struct timeval now;
         useconds_t diff;
         
         gettimeofday(&now, NULL);
         diff = usdiff(&lastPoll, &now);
         if(diff < pollDelayUsec)
             usleep(pollDelayUsec - diff);
         
         gettimeofday(&lastPoll, NULL);
     }
    
    uint8_t UltrasonicSensor::getStatus()
    {
        Telegram send, receive;
        unsigned char data[LS_STATUS_RESPONSE_LENGTH];
        int i = 0;
        
        create_lsGetStatus(&send, port);
        do
        {
            receive = brick->sendWithResponse(send);
            receive.getData(data);
            ++i;
        } while (data[3] == 0 && i < STATUS_TIMEOUT_COUNT);
        
        return data[3];
    }
    
    int msdiff(struct timeval * p_begin, struct timeval *p_end)
    {
        int result = (p_end->tv_sec - p_begin->tv_sec) * 1000;
        result += (p_end->tv_usec - p_begin->tv_usec) / 1000;
        return result;
    }
    
    useconds_t usdiff(struct timeval * p_begin, struct timeval *p_end)
    {
        useconds_t result = (p_end->tv_sec - p_begin->tv_sec) * 1000000;
        result += (p_end->tv_usec - p_begin->tv_usec);
        return result;
    }
}
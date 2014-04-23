#include <stdexcept>
#include "nxt_beagle/SensorThreading.hpp"
#include "nxt_beagle/UltraSensor.h"

#define DEF_SAMPLING_INTERVALL 100

namespace minotaur
{
    void *sensorThread(void *p_arg);
    
    SensorThreading::SensorThreading()
    {
        pthread_mutex_init(&evenSensorMutex, NULL);
        pthread_mutex_init(&unevenSensorMutex, NULL);
        
        run = true;
        publish = false;
        samplingIntervall = DEF_SAMPLING_INTERVALL;
        started = false;
    }
        
    void SensorThreading::setSensorPublisher(ros::Publisher *p_sensorPublisher)
    {
        pthread_mutex_lock(&unevenSensorMutex);
        pthread_mutex_lock(&evenSensorMutex);
        
        sensorDataPub = p_sensorPublisher;
        
        pthread_mutex_unlock(&evenSensorMutex);
        pthread_mutex_unlock(&unevenSensorMutex);
    }
    
    void SensorThreading::setPublish(const bool p_publish)
    {
        pthread_mutex_lock(&unevenSensorMutex);
        pthread_mutex_lock(&evenSensorMutex);
        
        pthread_mutex_unlock(&evenSensorMutex);
        pthread_mutex_unlock(&unevenSensorMutex);
    }
    
    void SensorThreading::setSamplingIntervall(const int p_samplingIntervall)
    {
        pthread_mutex_lock(&unevenSensorMutex);
        pthread_mutex_lock(&evenSensorMutex);
        
        
        
        pthread_mutex_unlock(&evenSensorMutex);
        pthread_mutex_unlock(&unevenSensorMutex);
    }
    
    uint8_t SensorThreading::addSensor(const uint8_t p_port)
    {
        pthread_mutex_lock(&unevenSensorMutex);
        pthread_mutex_lock(&evenSensorMutex);
        
        uint8_t result = sensorController.addSensor(p_port);
        
        pthread_mutex_unlock(&evenSensorMutex);
        pthread_mutex_unlock(&unevenSensorMutex);
        
        return result;
    }
    
    void SensorThreading::clearSensors()
    {
        pthread_mutex_lock(&unevenSensorMutex);
        pthread_mutex_lock(&evenSensorMutex);
        
        sensorController.clearSensors();
        
        pthread_mutex_unlock(&evenSensorMutex);
        pthread_mutex_unlock(&unevenSensorMutex);
    }
    
    uint8_t SensorThreading::getDistance(const uint8_t p_sensorID)
    {
        pthread_mutex_lock(&unevenSensorMutex);
        pthread_mutex_lock(&evenSensorMutex);
        
        uint8_t result = sensorController.getDistance(p_sensorID);
        
        pthread_mutex_unlock(&evenSensorMutex);
        pthread_mutex_unlock(&unevenSensorMutex);
        
        return result;
    }
    
    void *sensorThread(void *p_arg)
    {
        ros::Time begin, end;
        float sleepsec;
        nxt_beagle::UltraSensor msg;
        sensor_thread_arg * sensor_arg = (sensor_thread_arg*) p_arg;
        
        while(*(sensor_arg->run))
        {
            sleepsec = 0.1;
            
            if(*(sensor_arg->publish))
            {
                begin = ros::Time::now();
                pthread_mutex_lock(sensor_arg->mutex);
                for(int i = sensor_arg->start; i < sensor_arg->controller->sensorCount(); i += 2)
                {
                    msg.distance = sensor_arg->controller->getDistance(i);
                    (*(sensor_arg->sensorDataPub))->publish(msg);
                }
                
                end = ros::Time::now();
                
                sleepsec = ((float) (*(sensor_arg->samplingIntervall))) - ((float) (end.toSec() - begin.toSec()));
                
                pthread_mutex_unlock(sensor_arg->mutex);
            }
            
            ros::Duration(sleepsec).sleep();
        }
    }
    
    void SensorThreading::setBrick(nxtcon::Brick *p_brick)
    {
        pthread_mutex_lock(&unevenSensorMutex);
        pthread_mutex_lock(&evenSensorMutex);
        
        sensorController.setBrick(p_brick);
        
        pthread_mutex_unlock(&evenSensorMutex);
        pthread_mutex_unlock(&unevenSensorMutex);
    }
    
    void SensorThreading::start()
    {
        if(started)
            throw std::logic_error("SensorThreading cannot be started twice.");
        
        run = true;
        started = true;
        
        evenThread.controller = &sensorController;
        evenThread.start = 0;
        evenThread.mutex = &evenSensorMutex;
        evenThread.publish = &publish;
        evenThread.run = &run;
        evenThread.samplingIntervall = &samplingIntervall;
        
        unevenThread.controller = &sensorController;
        unevenThread.start = 1;
        unevenThread.mutex = &evenSensorMutex;
        unevenThread.publish = &publish;
        unevenThread.run = &run;
        unevenThread.samplingIntervall = &samplingIntervall;
        
        pthread_create(&threads[0], NULL, sensorThread, &evenThread);
        pthread_create(&threads[1], NULL, sensorThread, &unevenThread);
    }
    
     void SensorThreading::shutdown()
     {
         void *status;
         run = false;
         for(int i = 0; i < THREAD_COUNT; ++i)
             pthread_join(threads[i], &status);
     }
}
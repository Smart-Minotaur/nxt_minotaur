#ifndef NXT_BEAGLE_SENSOR_THREADING_HPP_
#define NXT_BEAGLE_SENSOR_THREADING_HPP_

#include <pthread.h>
#include <ros/ros.h>
#include "nxt_beagle/SensorController.hpp"

namespace minotaur
{
    typedef struct
    {
        SensorController *controller;
        int start;
        pthread_mutex_t *mutex;
        bool *publish;
        bool *run;
        int *samplingIntervall;
        ros::Publisher **sensorDataPub;
    } sensor_thread_arg;
   
    class SensorThreading
    {
    private:
        SensorController sensorController;
        
        static const unsigned int THREAD_COUNT = 2;
        
        pthread_t threads[THREAD_COUNT];
        pthread_mutex_t evenSensorMutex;
        pthread_mutex_t unevenSensorMutex;
        sensor_thread_arg evenThread;
        sensor_thread_arg unevenThread;
        
        int samplingIntervall;
        bool publish;
        bool run;
        bool started;
        
        ros::Publisher *sensorDataPub;
    public:
        
        SensorThreading();
        
        virtual ~SensorThreading() { }
        
        void setSensorPublisher(ros::Publisher *p_sensorPublisher);
        void setPublish(const bool p_publish);
        void setSamplingIntervall(const int p_samplingIntervall);
        
        uint8_t addSensor(const uint8_t p_port);
        void clearSensors();
        uint8_t getDistance(const uint8_t p_sensorID);
        void setBrick(nxtcon::Brick *p_brick);
        
        void start();
        void shutdown();
    };
}

#endif
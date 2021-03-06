#ifndef ROBOT_CONTROL_SENSOR_THREAD_HPP
#define ROBOT_CONTROL_SENSOR_THREAD_HPP

#include "robot_control/SensorCommunicator.hpp"
#include "minotaur_common/Thread.hpp"

namespace minotaur
{
     /**
     * \brief Creates a Thread to get sensorvalues from the brick
     * 
     */
    class SensorThread: public Thread
    {
    private:
        pthread_mutex_t intervalMutex;
        int samplingIntervalMsec;
        SensorCommunicator &sensorCommunicator;
        
        volatile bool keepRunning;
        
        void publishSensorData();
    protected:
        void onStart();
        void onStop();
    public:
        SensorThread(SensorCommunicator &p_sensorCommunicator);
        ~SensorThread();
        
	/**
	 * Control-loop until onStop() is called
	 */
        void run();
        
        void setSamplingInterval(const int p_samplingIntervalMsec);
        int getSamplingInterval();
    };
}

#endif

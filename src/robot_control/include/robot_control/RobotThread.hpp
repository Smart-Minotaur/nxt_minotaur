#ifndef ROBOT_CONTROL_ROBOT_THREAD_HPP
#define ROBOT_CONTROL_ROBOT_THREAD_HPP

#include "robot_control/RobotCommunicator.hpp"
#include "minotaur_common/Thread.hpp"

namespace minotaur
{
    
    class RobotThread: public Thread
    {
    private:
        pthread_mutex_t intervalMutex;
        int samplingIntervalMsec;
        RobotCommunicator &robotCommunicator;
        
        volatile bool keepRunning;
        
        void stepRobotController(const int p_samplingInterval);
    protected:
        void onStart();
        void onStop();
    public:
        RobotThread(RobotCommunicator &p_robotCommunicator);
        ~RobotThread();
        
        void run();
        
        void setSamplingInterval(const int p_samplingIntervalMsec);
        int getSamplingInterval();
    };
}

#endif

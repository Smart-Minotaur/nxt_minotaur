#ifndef ROBOT_CONTROL_ROBOT_THREAD_HPP
#define ROBOT_CONTROL_ROBOT_THREAD_HPP

#include "robot_control/RobotCommunicator.hpp"
#include "minotaur_common/Thread.hpp"

namespace minotaur
{
    /**
     * \brief Creates a Thread to start the robot
     * 
     */
    class RobotThread: public Thread
    {
    private:
        pthread_mutex_t intervalMutex;
        int samplingIntervalMsec;
        RobotCommunicator &robotCommunicator;
        
        volatile bool keepRunning;
        
        void stepRobotController(const int p_samplingInterval);
    protected:
      /**
       * Start the robot
       */
        void onStart();
	/**
	 * Terminates the robot-thread
	 */
        void onStop();
    public:
        RobotThread(RobotCommunicator &p_robotCommunicator);
        ~RobotThread();
        
	/**
	 * Control-loop for the robot until onStop() is called
	 */
        void run();
        
        void setSamplingInterval(const int p_samplingIntervalMsec);
        int getSamplingInterval();
    };
}

#endif

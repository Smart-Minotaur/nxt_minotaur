#ifndef MINOTAUR_NAVIGATION_THREAD_HPP_
#define MINOTAUR_NAVIGATION_THREAD_HPP_

#include <pthread.h>
#include "minotaur_pc/MinotaurCommunicator.hpp"
#include "minotaur_pc/MinotaurThread.hpp"
#include "minotaur_pc/RobotOdometry.hpp"

namespace minotaur
{
    class NavigationThread : public MinotaurThread
    {
    private:
        pthread_t thread;
        MinotaurCommunicator *communicator;
        volatile bool keepRunning;
        RobotOdometry* odom;
    public:
        NavigationThread() { keepRunning = true;}
        virtual ~NavigationThread() { }
        
        void init(MinotaurCommunicator *p_communicator, RobotOdometry* p_odom);
        
        int run();
        void setRun(const bool p_run);
    };
}

#endif
#ifndef MINOTAUR_MAP_NAVIGATION_THREAD_HPP_
#define MINOTAUR_MAP_NAVIGATION_THREAD_HPP_

#include <pthread.h>
#include "minotaur_map/MinotaurCommunicator.hpp"
#include "minotaur_map/RobotOdometry.hpp"
#include "minotaur_common/Thread.hpp"

namespace minotaur
{
    /**
     * \brief Runs the movement and navigation logic of the robot.
     */
    class NavigationThread : public Thread
    {
    private:
        MinotaurCommunicator *communicator;
        volatile bool keepRunning;
        RobotOdometry* odom;
    protected:
        void onStop();
        void onStart();
    public:
        NavigationThread();
        ~NavigationThread();
        
        void init(MinotaurCommunicator *p_communicator, RobotOdometry* p_odom);
        
        void run();
    };
}

#endif

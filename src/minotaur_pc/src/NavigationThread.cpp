#include <ros/ros.h>
#include "minotaur_pc/NavigationThread.hpp"

#define NAV_FREQUENCY 10

namespace minotaur
{
    void NavigationThread::init(MinotaurCommunicator *p_communicator, RobotOdometry* p_odom)
    {
        communicator = p_communicator;
        odom = p_odom;
        keepRunning = true;
    }
        
    int NavigationThread::run()
    {
        ros::Rate loopRate(NAV_FREQUENCY);
        RobotPosition target;
        while(keepRunning)
        {
            //algorithm.setPosition(state->getPosition);
            //target = algorithm.getTarget();
            communicator->setTargetPosition(target);
            loopRate.sleep();
        }
        
        return 0;
    }
    
    void NavigationThread::setRun(const bool p_run)
    {
        keepRunning = p_run;
    }
}
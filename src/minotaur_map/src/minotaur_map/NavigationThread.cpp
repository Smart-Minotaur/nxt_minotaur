#include <ros/ros.h>
#include "minotaur_map/NavigationThread.hpp"

#define NAV_FREQUENCY 10

namespace minotaur
{
    NavigationThread::NavigationThread()
    {
        
    }
    NavigationThread::~NavigationThread()
    {
        
    }
    
    void NavigationThread::init(MinotaurCommunicator *p_communicator, RobotOdometry* p_odom)
    {
        communicator = p_communicator;
        odom = p_odom;
    }
    
    void NavigationThread::onStart()
    {
        keepRunning = true;
    }
    
    void NavigationThread::onStop()
    {
        keepRunning = false;
    }
        
    void NavigationThread::run()
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
    }
}

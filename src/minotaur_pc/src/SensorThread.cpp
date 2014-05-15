#include "minotaur_pc/SensorThread.hpp"

namespace minotaur
{
    
    void SensorThread::init(RobotOdometry* p_odom, BlockingQueue<SensorMeasurement> *p_queue)
    {
        queue = p_queue;
        odom = p_odom;
        keepRunning = true;
    }
    
    int SensorThread::run()
    {
        while(keepRunning)
        {
            queue->dequeue();
            //mapCreator.setPosition(odom->getPosition);
            //mapCreator.step(sensorMeasurement);
        }
        
        return 0;
    }
    
    void SensorThread::setRun(const bool p_run)
    {
        keepRunning = p_run;
    }
}
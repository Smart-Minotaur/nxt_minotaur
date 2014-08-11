#include "minotaur_map/SensorThread.hpp"

namespace minotaur
{
    SensorThread::SensorThread()
    {
        
    }
    
    SensorThread::~SensorThread()
    {
        
    }
    
    void SensorThread::init(RobotOdometry* p_odom, BlockingQueue<SensorMeasurement> *p_queue)
    {
        queue = p_queue;
        odom = p_odom;
    }
    
    void SensorThread::onStart()
    {
        keepRunning = true;
    }
    
    void SensorThread::onStop()
    {
        keepRunning = false;
    }
    
    void SensorThread::run()
    {
        while(keepRunning)
        {
            queue->dequeue();
            //mapCreator.setPosition(odom->getPosition);
            //mapCreator.step(sensorMeasurement);
        }
    }
    
}

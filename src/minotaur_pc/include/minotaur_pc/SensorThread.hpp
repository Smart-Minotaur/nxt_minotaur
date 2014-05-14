#ifndef MINOTAUR_PC_SENSOR_THREAD_HPP_
#define MINOTAUR_PC_SENSOR_THREAD_HPP_

#include "minotaur_pc/MinotaurThread.hpp"
#include "minotaur_pc/BlockingQueue.hpp"
#include "minotaur_pc/SensorMeasurement.hpp"
#include "minotaur_pc/RobotOdometry.hpp"

namespace minotaur
{
    class SensorThread : public MinotaurThread
    {
    private:
        BlockingQueue<SensorMeasurement> *queue;
        volatile bool keepRunning;
        RobotOdometry* odom;
    public:
        SensorThread() { }
        virtual ~SensorThread() { }
        
        void init(RobotOdometry* p_odom, BlockingQueue<SensorMeasurement> *p_queue);
        int run();
        void setRun(const bool p_run);
    };
}

#endif
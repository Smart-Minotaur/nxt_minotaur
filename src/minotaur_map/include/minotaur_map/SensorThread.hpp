#ifndef MINOTAUR_MAP_SENSOR_THREAD_HPP_
#define MINOTAUR_MAP_SENSOR_THREAD_HPP_

#include "minotaur_map/SensorMeasurement.hpp"
#include "minotaur_map/RobotOdometry.hpp"
#include "minotaur_common/BlockingQueue.hpp"
#include "minotaur_common/Thread.hpp"

namespace minotaur
{
    /**
     * \brief Runs the sensor processing logic of the robot.
     */
    class SensorThread : public Thread
    {
    private:
        BlockingQueue<SensorMeasurement> *queue;
        volatile bool keepRunning;
        RobotOdometry* odom;
    protected:
        void onStop();
        void onStart();
    public:
        SensorThread();
        ~SensorThread();
                
        void init(RobotOdometry* p_odom, BlockingQueue<SensorMeasurement> *p_queue);
        void run();
    };
}

#endif

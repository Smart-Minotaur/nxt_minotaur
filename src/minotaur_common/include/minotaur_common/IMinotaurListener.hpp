#ifndef MINOTAUR_MINOTAUR_LISTENER_HPP
#define MINOTAUR_MINOTAUR_LISTENER_HPP

#include <nav_msgs/Odometry.h>
#include "minotaur_common/UltrasonicData.h"

namespace minotaur
{   
    /**
     * \brief The IMinotaurListener class provides an Interface to react on
     *        incoming Odometry and UltrasonicData messages within a
     *        MinotaurControlNode object. 
     */
    class IMinotaurListener
    {
    public:
    
        /**
         * This method is called when an incoming Odometry message is
         * received.
         * @param p_odometry the incoming Odometry message
         */
        virtual void onReceiveOdometry(const nav_msgs::Odometry &p_odometry) = 0;
        
        /**
         * This method is called when an incoming UltrasonicData message
         * is received.
         * @param p_sensorData the incoming UltrasonicData message
         */
        virtual void onReceiveUltrasonicData(const minotaur_common::UltrasonicData &p_sensorData) = 0;
    };
}

#endif

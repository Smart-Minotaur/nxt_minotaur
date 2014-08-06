#ifndef MINOTAUR_MINOTAUR_CONTROL_NODE_HPP
#define MINOTAUR_MINOTAUR_CONTROL_NODE_HPP

#include <ros/ros.h>
#include <pthread.h>
#include <nav_msgs/Odometry.h>
#include "minotaur_common/UltrasonicData.h"
#include "minotaur_common/IMinotaurListener.hpp"

namespace minotaur
{
    class DefaultMinotaurListener : public IMinotaurListener
    {
    public:
        void onReceiveOdometry(const nav_msgs::Odometry &p_odometry) { }
        void onReceiveUltrasonicData(const minotaur_common::UltrasonicData &p_sensorData) { }
    };
    
    /**
     * \brief The MinotaurControlNode class is used to control the
     *        minotaur via ROS.
     * 
     * It is an abstraction of the ROS communication, therefore no more
     * ROS logic is needed to control the minotaur.
     */
    class MinotaurControlNode
    {
    private:
        volatile bool keepRunning;
    
        ros::Subscriber odometrySubscriber;
        ros::Subscriber ultrasonicSubscriber;
        
        ros::Publisher robotVelocityPublisher;
        ros::Publisher pidParamPublisher;
        ros::Publisher targetPosPub;
        
        nav_msgs::Odometry lastOdometry;
        pthread_mutex_t odomMutex;
        
        DefaultMinotaurListener defaultListener;
        IMinotaurListener *listener;
        
        bool connected;
        
        void processOdometryMsg(const nav_msgs::Odometry &p_msg);
        void updateLastOdometry(const nav_msgs::Odometry &p_msg);
        double getThetaFromLastOdom();
        void processSensorMsg(const minotaur_common::UltrasonicData &p_msg);
        
    public:
        MinotaurControlNode(): connected(false), listener(&defaultListener) { pthread_mutex_init(&odomMutex, NULL); }
        ~MinotaurControlNode() { pthread_mutex_destroy(&odomMutex); }
        
        /**
         * This method must be called before any other methods can be
         * used regarding ROS communication.
         * @param p_nodeHandle the ros node handle to set up communication
         */
        void connectToROS(ros::NodeHandle &p_nodeHandle);
        
        void setVelocity(const float p_linearVelocity, const float p_angularVelocity);
        void setPIDParameter(const float p_Kp, const float p_Ki, const float p_Kd);
        void setSimpleTarget(const float p_x, const float p_y, const float p_theta);
        
        /**
         * This method computes incoming and sends outgoing messages.
         * Receiving and sending messages is handled in the same thread.
         */
        void spin();
        void stop();
        
        /**
         * Set a IMinotaurListener to react on incoming messages.
         * @param p_listener listener to handle messages
         */
        void setMinotaurListener(IMinotaurListener *p_listener);
    };
}

#endif

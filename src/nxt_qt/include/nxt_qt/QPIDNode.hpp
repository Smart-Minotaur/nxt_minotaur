#ifndef QPID_NODE_HPP_
#define QPID_NODE_HPP_

#include <string>
#include <ros/ros.h>
#include <QThread>
#include <QMetaType>
#include <QMutex>
#include "nxt_beagle/MotorVelocity.hpp"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"

namespace minotaur
{
    class QRobotVelocity
    {
    public:
        float linearVelocity;
        float angularVelocity;
        
        QRobotVelocity()
        :linearVelocity(0), angularVelocity(0) { }
        QRobotVelocity(const float p_linVel, const float p_angVel)
        :linearVelocity(p_linVel), angularVelocity(p_angVel) { }
        virtual ~QRobotVelocity() { }
    };
    
    class QPIDNode : public QThread
    {
        Q_OBJECT
        
    private:
        QMutex mutex;
        
        ros::NodeHandle nodeHandle;
        
        ros::Subscriber odometrySub;
        
        ros::Publisher samplingIntervalPublisher;
        ros::Publisher robotVelocityPublisher;
        ros::Publisher pidPramPublisher;
        ros::Publisher setModelPublisher;
        
        nav_msgs::Odometry lastOdometry;
        
        void processOdometryMsg(const nav_msgs::Odometry& p_msg);
        
    public:
        QPIDNode();
        virtual ~QPIDNode() { }
        
        void setSamplingInterval(const int p_msec);
        void setRobotVelocity(const float p_linVel, const float p_angVel);
        void setPIDParameter(const float p_Kp, const float p_Ki, const float p_Kd);
        void setModel(const std::string& p_name);
        
        void run();
        
    Q_SIGNALS:
        void rosShutdown();
        void measuredVelocityUpdated(const QRobotVelocity p_msg);
    };
}

Q_DECLARE_METATYPE(minotaur::QRobotVelocity);

#endif
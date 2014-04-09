#ifndef QPID_NODE_HPP_
#define QPID_NODE_HPP_

#include <ros/ros.h>
#include <QThread>
#include <QMetaType>
#include "nxt_beagle/MVelocity.h"
#include "nxt_beagle/MotorVelocity.hpp"

namespace minotaur
{
    class QMotorVelocity : public MotorVelocity
    {
    public:
        QMotorVelocity()
        :MotorVelocity() { }
        QMotorVelocity(const float p_left, const float p_right)
        :MotorVelocity(p_left, p_right) { }
        QMotorVelocity(const QMotorVelocity& p_velocity)
        :MotorVelocity(p_velocity) { }
        virtual ~QMotorVelocity() { }
    };
    
    class QPIDNode : public QThread
    {
        Q_OBJECT
        
    private:
        ros::NodeHandle nodeHandle;
        
        ros::Subscriber targetMVelSubscriber;
        ros::Subscriber measuredMVelSubscriber;
        
        ros::Publisher samplingIntervalPublisher;
        ros::Publisher robotVelocityPublisher;
        ros::Publisher pidPramPublisher;
        
        void processTargetMotorVelocity(const nxt_beagle::MVelocity& p_msg);
        void processMeasuredMotorVelocity(const nxt_beagle::MVelocity& p_msg);
        
    public:
        QPIDNode();
        virtual ~QPIDNode() { }
        
        void setSamplingInterval(const int p_msec);
        void setRobotVelocity(const float p_linVel, const float p_angVel);
        void setPIDParameter(const float p_Kp, const float p_Ki, const float p_Kd);
        
        void run();
        
    Q_SIGNALS:
        void rosShutdown();
        void targetMotorVelocityUpdated(const QMotorVelocity p_msg);
        void measuredMotorVelocityUpdated(const QMotorVelocity p_msg);
    };
}

Q_DECLARE_METATYPE(minotaur::QMotorVelocity);

#endif
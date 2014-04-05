#ifndef QPID_NODE_HPP_
#define QPID_NODE_HPP_

#include <ros/ros.h>
#include <QThread>
#include "nxt_beagle/MVelocity.h"

namespace minotaur
{
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
        
        void setSamplingIntervall(const float p_sec);
        void setRobotVelocity(const float p_linVel, const float p_angVel);
        void setPIDParameter(const float p_Kp, const float p_Ki, const float p_Kd);
        
        void run();
        
    Q_SIGNALS:
        void rosShutdown();
        void targetMotorVelocityUpdated(const nxt_beagle::MVelocity& p_msg);
        void measuredMotorVelocityUpdated(const nxt_beagle::MVelocity& p_msg);
    };
}

#endif
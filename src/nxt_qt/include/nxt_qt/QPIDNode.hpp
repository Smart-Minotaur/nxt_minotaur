#ifndef QPID_NODE_HPP_
#define QPID_NODE_HPP_

#include <string>
#include <ros/ros.h>
#include <QThread>
#include <QMetaType>
#include <QMutex>
#include <vector>
#include "nxt_beagle/MotorVelocity.hpp"
#include "nxt_beagle/UltraSensor.h"
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
    
    class QUltraSensor
    {
    public:
        int id;
        int distance;
        float direction;
        
        QUltraSensor() { }
        virtual ~QUltraSensor() { }
    };
    
    class QPIDNode : public QThread
    {
        Q_OBJECT
        
    private:
        QMutex mutex;
        
        std::vector<float> directions;
        
        ros::NodeHandle nodeHandle;
        
        ros::Subscriber odometrySub;
        ros::Subscriber ultrasensorSub;
        
        ros::Publisher robotVelocityPublisher;
        ros::Publisher pidPramPublisher;
        
        nav_msgs::Odometry lastOdometry;
        
        void processOdometryMsg(const nav_msgs::Odometry& p_msg);
        void processSensorMsg(const nxt_beagle::UltraSensor p_msg);
    public:
        QPIDNode();
        virtual ~QPIDNode() { }
        
        void setRobotVelocity(const float p_linVel, const float p_angVel);
        void setPIDParameter(const float p_Kp, const float p_Ki, const float p_Kd);
        
        void run();
        
    Q_SIGNALS:
        void rosShutdown();
        void measuredVelocityUpdated(const QRobotVelocity p_msg);
        void measuredSensor(const QUltraSensor p_msg);
    };
}

Q_DECLARE_METATYPE(minotaur::QRobotVelocity);
Q_DECLARE_METATYPE(minotaur::QUltraSensor);

#endif
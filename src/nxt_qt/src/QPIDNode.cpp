#include "nxt_beagle/Config.hpp"
#include "nxt_qt/QPIDNode.hpp"
#include "nxt_beagle/Config.hpp"
#include "nxt_beagle/SamplingInterval.h"
#include "nxt_beagle/RVelocity.h"
#include "nxt_beagle/PIDParam.h"

namespace minotaur
{
    QPIDNode::QPIDNode()
    {
        ROS_INFO("Subscribing to topic \"%s\"...", NXT_TARGET_MOTOR_VELOCITY_TOPIC);
        targetMVelSubscriber = nodeHandle.subscribe(NXT_TARGET_MOTOR_VELOCITY_TOPIC,
                                                    1000,
                                                    &QPIDNode::processTargetMotorVelocity,
                                                    this);
        ROS_INFO("Subscribing to topic \"%s\"...", NXT_MEASURE_MOTOR_VELOCITY_TOPIC);
        measuredMVelSubscriber = nodeHandle.subscribe(NXT_MEASURE_MOTOR_VELOCITY_TOPIC,
                                                      1000,
                                                      &QPIDNode::processMeasuredMotorVelocity,
                                                      this);
        
        ROS_INFO("Publishing on topic \"%s\"...", NXT_SET_SAMPLING_INTERVAL_TOPIC);
        samplingIntervalPublisher = nodeHandle.advertise<nxt_beagle::SamplingInterval>(NXT_SET_SAMPLING_INTERVAL_TOPIC, 1000);
        ROS_INFO("Publishing on topic \"%s\"...", NXT_SET_ROBOT_VELOCITY_TOPIC);
        robotVelocityPublisher = nodeHandle.advertise<nxt_beagle::RVelocity>(NXT_SET_ROBOT_VELOCITY_TOPIC, 1000);
        ROS_INFO("Publishing on topic \"%s\"...", NXT_SET_PID_PARAMETER);
        pidPramPublisher = nodeHandle.advertise<nxt_beagle::PIDParam>(NXT_SET_PID_PARAMETER, 1000);
    }
    
    void QPIDNode::setSamplingInterval(const int p_msec)
    {
        ROS_INFO("Sending Interval: %d msec.", p_msec);
        nxt_beagle::SamplingInterval msg;
        msg.msec = p_msec;
        samplingIntervalPublisher.publish(msg);
    }
    
    void QPIDNode::setRobotVelocity(const float p_linVel, const float p_angVel)
    {
        nxt_beagle::RVelocity msg;
        msg.linearVelocity = p_linVel;
        msg.angularVelocity = p_angVel;
        robotVelocityPublisher.publish(msg);
    }
    
    void QPIDNode::setPIDParameter(const float p_Kp, const float p_Ki, const float p_Kd)
    {
        nxt_beagle::PIDParam msg;
        msg.Kp = p_Kp;
        msg.Ki = p_Ki;
        msg.Kd = p_Kd;
        pidPramPublisher.publish(msg);
    }
    
    void QPIDNode::run()
    {
        ros::spin();
        Q_EMIT rosShutdown();
    }
    
    void QPIDNode::processTargetMotorVelocity(const nxt_beagle::MVelocity& p_msg)
    {
       QMotorVelocity vel(p_msg.leftVelocity, p_msg.rightVelocity);
       Q_EMIT targetMotorVelocityUpdated(vel);
    }
    
    void QPIDNode::processMeasuredMotorVelocity(const nxt_beagle::MVelocity& p_msg)
    {
        QMotorVelocity vel(p_msg.leftVelocity, p_msg.rightVelocity);
        Q_EMIT measuredMotorVelocityUpdated(vel);
    }
}
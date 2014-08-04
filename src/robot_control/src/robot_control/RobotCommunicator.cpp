#include "robot_control/RobotCommunicator.hpp"
#include "nxt_control/NxtOpcodes.hpp"
#include "minotaur_common/MinotaurTopics.hpp"
#include "minotaur_common/RAIILock.hpp"

#define WHEEL_TRACK 0.12f
#define WHEEL_RADIUS 0.025f
#define LEFT_PORT PORT_A
#define RIGHT_PORT PORT_B

#define ROS_MSG_QUEUE_LENGTH 20

namespace minotaur
{
    RobotCommunicator::RobotCommunicator()
    {
        pthread_mutex_init(&robotMutex, NULL);
    }
    
    RobotCommunicator::~RobotCommunicator()
    {
        pthread_mutex_destroy(&robotMutex);
    }
    
    void RobotCommunicator::init(ros::NodeHandle &p_handle, nxtcon::Brick *p_brick)
    {
        RAIILock lock(&robotMutex);
        leftMotor.setBrick(p_brick);
        leftMotor.setPort(LEFT_PORT);
        rightMotor.setBrick(p_brick);
        rightMotor.setPort(RIGHT_PORT);
        
        ROS_INFO("-- Setting up RobotController.");
        robotController.setWheelTrack(WHEEL_TRACK);
        robotController.getPIDController().setWheelRadius(WHEEL_RADIUS);
        robotController.getPIDController().setLeftMotor(&leftMotor);
        robotController.getPIDController().setRightMotor(&rightMotor);
        
        ROS_INFO("-- Subscribing to topic \"%s\".", MINOTAUR_SET_PID_PARAMETER_TOPIC);
        setPIDParamSub = p_handle.subscribe(MINOTAUR_SET_PID_PARAMETER_TOPIC, ROS_MSG_QUEUE_LENGTH, &RobotCommunicator::processPIDParamMsg, this);
        ROS_INFO("-- Subscribing to topic \"%s\".", ROS_VEL_TOPIC);
        cmdVelSub = p_handle.subscribe(ROS_VEL_TOPIC, ROS_MSG_QUEUE_LENGTH, &RobotCommunicator::processSetVelocityMsg, this);
        
        ROS_INFO("-- Publishing on topic \"%s\".", ROS_ODOM_TOPIC);
        odometryPub = p_handle.advertise<nav_msgs::Odometry>(ROS_ODOM_TOPIC, ROS_MSG_QUEUE_LENGTH);
    }
    
    void RobotCommunicator::setTransformBroadcaster(tf::TransformBroadcaster *p_odomBroadcaster)
    {
        RAIILock lock(&robotMutex);
        odomBroadcaster = p_odomBroadcaster;
    }
        
    void RobotCommunicator::publish()
    {
        RAIILock lock(&robotMutex);
        nav_msgs::Odometry odom = robotController.getOdometry();
        ros::Time currentTime = ros::Time::now();
        
        // create Transformation and send it
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = currentTime;
        odom_trans.header.frame_id = MINOTAUR_ODOM_FRAME;
        odom_trans.child_frame_id = MINOTAUR_BASE_FRAME;

        odom_trans.transform.translation.x = odom.pose.pose.position.x;
        odom_trans.transform.translation.y = odom.pose.pose.position.y;
        odom_trans.transform.translation.z = odom.pose.pose.position.z;
        odom_trans.transform.rotation = odom.pose.pose.orientation;

        odomBroadcaster->sendTransform(odom_trans);
        
        // prepare odometry message and send it
        odom.header.stamp = currentTime;
        odom.header.frame_id = MINOTAUR_ODOM_FRAME;
        odom.child_frame_id = MINOTAUR_BASE_FRAME;
        
        odometryPub.publish(odom);
    }
    
    void RobotCommunicator::stepController(const int p_samplingIntervalMsec)
    {
        RAIILock lock(&robotMutex);
        robotController.step(p_samplingIntervalMsec);
    }
    
    /* callbacks */
    void RobotCommunicator::processSetVelocityMsg(const geometry_msgs::Twist& p_msg)
    {
        RAIILock lock(&robotMutex);
        robotController.setVelocity(p_msg);
    }
    
    void RobotCommunicator::processPIDParamMsg(const minotaur_common::PIDParameter& p_msg)
    {
        ROS_INFO("Changing PIDParameter to: Kp = %.4f; Ki = %.4f; Kd = %.4f.",p_msg.Kp, p_msg.Ki, p_msg.Kd);  
        
        RAIILock lock(&robotMutex);
        robotController.getPIDController().setPIDParameter(p_msg);
    }
    
    void RobotCommunicator::applySettings(const RobotSettings &p_settings)
    {
        RAIILock lock(&robotMutex);
        robotController.getPIDController().setPIDParameter(p_settings.pidParameter);
        robotController.getPIDController().setWheelRadius(p_settings.wheelRadius);
        robotController.setWheelTrack(p_settings.wheelTrack);
    }
}

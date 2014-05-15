/*
 * Author: Fabian Meyer
 */

#ifndef NXT_BEAGLE_CONFIG_HPP
#define NXT_BEAGLE_CONFIG_HPP

#include <string>
#include <sstream>

#define NXT_BEAGLE_MAJOR_VERSION 0
#define NXT_BEAGLE_MINOR_VERSION 1
#define NXT_BEAGLE_PATCH_VERSION 0
#define NXT_BEAGLE_VERSION 0.1.0

#define NXT_PORT1 0
#define NXT_PORT2 1
#define NXT_PORT3 2
#define NXT_PORT4 3

#define MINOTAUR_ROS_NS "/minotaur"

#define HERACLES_NAME "Heracles"
#define THESEUS_NAME "Theseus"
#define CURRENT_MODEL_NAME "CurrentModel"

#define NXT_ULTRA_SENSOR_TOPIC MINOTAUR_ROS_NS "/measure_ultrasensor"
#define NXT_SET_PID_PARAMETER MINOTAUR_ROS_NS "/set_pid_param"
#define NXT_CLEAR_SENSOR_TOPIC MINOTAUR_ROS_NS "/clear_sensor"

#define NXT_GET_TICKS_SRV MINOTAUR_ROS_NS "/get_ticks"
#define NXT_GET_ULTRASONIC_SRV MINOTAUR_ROS_NS "/get_ultrasonic"
#define NXT_ADD_ULTRASONIC_SRV MINOTAUR_ROS_NS "/add_ultrasonic"

#define ROS_VEL_TOPIC "/cmd_vel"
#define ROS_ODOM_TOPIC "/odom"
#define ROS_SIMPLE_GOAL "/move_base_simple/goal"

/* Frame names */
#define MINOTAUR_ODOM_FRAME "odom"
#define MINOTAUR_BASE_FRAME "base_link"

inline std::string PARAM_NAME(const std::string& p_name)
{ return (MINOTAUR_ROS_NS "/") + p_name; }

inline std::string PARAM_WHEEL_TRACK(const std::string& p_name)
{ return PARAM_NAME(p_name) + "/wheel_track"; }

inline std::string PARAM_WHEEL_CIRCUMFERENCE(const std::string& p_name)
{ return PARAM_NAME(p_name) + "/wheel_circumference"; }

inline std::string PARAM_KP(const std::string& p_name)
{ return PARAM_NAME(p_name) + "/kp"; }

inline std::string PARAM_KI(const std::string& p_name)
{ return PARAM_NAME(p_name) + "/ki"; }

inline std::string PARAM_KD(const std::string& p_name)
{ return PARAM_NAME(p_name) + "/kd"; }

inline std::string PARAM_SAMPLING_INTERVAL(const std::string& p_name)
{ return PARAM_NAME(p_name) + "/sampling_interval"; }

inline std::string PARAM_CURRENT_MODEL()
{ return PARAM_NAME(CURRENT_MODEL_NAME);}

inline std::string PARAM_SENSOR(const std::string& p_name, const int p_id)
{   
    std::stringstream ss;
    ss << "/sensor_" << p_id;
    return PARAM_NAME(p_name) + ss.str();
}

inline std::string PARAM_SENSOR_DX(const std::string& p_name, const int p_id)
{ return PARAM_SENSOR(p_name, p_id) + "/dx"; }

inline std::string PARAM_SENSOR_DY(const std::string& p_name, const int p_id)
{ return PARAM_SENSOR(p_name, p_id) + "/dy"; }

inline std::string PARAM_SENSOR_DIRECTION(const std::string& p_name, const int p_id)
{ return PARAM_SENSOR(p_name, p_id) + "/direction"; }

#endif


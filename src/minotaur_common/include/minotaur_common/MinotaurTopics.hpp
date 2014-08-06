/**
 * \file MinotaurParam.hpp
 * \brief Contains symbols for the ROS communication (topics, services).
 */

#ifndef MINOTAUR_TOPICS_HPP
#define MINOTAUR_TOPICS_HPP

#define SENSOR_PORT1 0
#define SENSOR_PORT2 1
#define SENSOR_PORT3 2
#define SENSOR_PORT4 3

#define MINOTAUR_TOPIC_NS "/minotaur"

#define MINOTAUR_MEASURE_SENSOR_TOPIC MINOTAUR_TOPIC_NS "/measure_ultrasensor"
#define MINOTAUR_SET_PID_PARAMETER_TOPIC MINOTAUR_TOPIC_NS "/set_pid_param"
#define MINOTAUR_CLEAR_SENSOR_TOPIC MINOTAUR_TOPIC_NS "/clear_sensor"

#define MINOTAUR_GET_TICKS_SRV MINOTAUR_TOPIC_NS "/get_ticks"
#define MINOTAUR_GET_ULTRASONIC_SRV MINOTAUR_TOPIC_NS "/get_ultrasonic"
#define MINOTAUR_ADD_ULTRASONIC_SRV MINOTAUR_TOPIC_NS "/add_ultrasonic"

#define ROS_VEL_TOPIC "/cmd_vel"
#define ROS_ODOM_TOPIC "/odom"
#define ROS_SIMPLE_GOAL "/move_base_simple/goal"

/* Frame names */
#define MINOTAUR_ODOM_FRAME "odom"
#define MINOTAUR_BASE_FRAME "base_link"

#endif

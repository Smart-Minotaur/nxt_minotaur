/**
 * Global config file for the MouseMonitor which is used in qt and on
 * the beagle side.
 */

#ifndef MOUSE_MONITOR_CONFIG_H
#define MOUSE_MONITOR_CONFIG_H

// Messages
#include "mouse_monitor_beagle/MouseMonitorSensorData.h"
#include "mouse_monitor_beagle/MouseMonitorSensorSettings.h"
#include "mouse_monitor_beagle/MouseMonitorSensorGetSettings.h"
#include "mouse_monitor_beagle/MouseMonitorSensorGetData.h"
#include "mouse_monitor_beagle/MouseMonitorSensorSetResolution.h"

#define SENSOR1 "/dev/spidev1.0"
#define SENSOR2 "/dev/spidev1.1"

#define SENSOR_SAMPLE_FREQUENCY 1000 // Hz

#define ROS_MOUSE_DATA_TOPIC "/mouseData"
#define ROS_MOUSE_SETTINGS_TOPIC "/mouseSettings"

#define NODE_NAME_QT "MouseMonitor"
#define NODE_NAME_BEAGLE "MouseMonitorBeagle"

// TODO topics nach minotaur_common/MinotaurTopics.hpp verschieben

#endif

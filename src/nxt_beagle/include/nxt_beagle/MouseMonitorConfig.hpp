/**
 * Global config file for the MouseMonitor which is used in qt and on
 * the beagle side.
 */

#ifndef MOUSE_MONITOR_CONFIG_H
#define MOUSE_MONITOR_CONFIG_H

// Messages
#include "nxt_beagle/MouseMonitorSensorData.h"
#include "nxt_beagle/MouseMonitorSensorSettings.h"
#include "nxt_beagle/MouseMonitorSensorGetSettings.h"
#include "nxt_beagle/MouseMonitorSensorGetData.h"
#include "nxt_beagle/MouseMonitorSensorSetResolution.h"

#define SENSOR1 "/dev/spidev1.0"
#define SENSOR2 "/dev/spidev1.1"

#define ROS_MOUSE_DATA_TOPIC "/mouseData"
#define ROS_MOUSE_SETTINGS_TOPIC "/mouseSettings"

#define NODE_NAME_QT "MouseMonitor"
#define NODE_NAME_BEAGLE "MouseMonitorBeagle"

#endif

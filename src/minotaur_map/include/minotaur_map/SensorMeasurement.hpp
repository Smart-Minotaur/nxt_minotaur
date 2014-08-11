#ifndef MINOTAUR_MAP_SENSOR_MEASUREMENT_HPP_
#define MINOTAUR_MAP_SENSOR_MEASUREMENT_HPP_

#include <stdint.h>
#include "minotaur_map/Vec2.hpp"

namespace minotaur
{
    class SensorMeasurement
    {
    public:
        int sensorID;
        uint8_t distance;
        
        SensorMeasurement() { }
        SensorMeasurement(const int p_sensorID, const uint8_t distance) { }
        virtual ~SensorMeasurement() { }
    };
}

#endif

#ifndef MINOTAUR_PC_SENSOR_MEASUREMENT_HPP_
#define MINOTAUR_PC_SENSOR_MEASUREMENT_HPP_

#include "minotaur_pc/Vec2.hpp"
#include <stdint.h>

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
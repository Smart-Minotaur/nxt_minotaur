#ifndef MINOTAUR_PC_SENSOR_MEASUREMENT_HPP_
#define MINOTAUR_PC_SENSOR_MEASUREMENT_HPP_

#include "minotaur_pc/Vec2.hpp"
#include <stdint.h>

namespace minotaur
{
    class SensorMeasurement
    {
    public:
        Vec2 offset;
        float direction;
        uint8_t distance;
        
        SensorMeasurement() { }
        virtual ~SensorMeasurement() { }
    };
}

#endif
#ifndef MINOTAUR_SENSOR_SETTING_H
#define MINOTAUR_SENSOR_SETTING_H

namespace minotaur
{
    class SensorSetting
    {
    public:
        int id;
        float direction;
        float x, y;
        
        SensorSetting() { }
        SensorSetting(const int p_id, const float p_direction, const float p_x, const float p_y)
        : id(p_id), direction(p_direction), x(p_x), y(p_y) { }
        ~SensorSetting() { }
    }
}

#endif
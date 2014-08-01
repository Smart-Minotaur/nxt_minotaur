#ifndef MINOTAUR_SENSOR_SETTINGS_HPP
#define MINOTAUR_SENSOR_SETTINGS_HPP

#include <vector>
#include <string>

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
    };
    
    std::vector<SensorSetting> loadSensorSettings(const std::string &p_modelName);
    std::vector<SensorSetting> loadCurrentSensorSettings();
}

#endif

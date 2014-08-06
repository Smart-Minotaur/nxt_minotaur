#ifndef MINOTAUR_SENSOR_SETTINGS_HPP
#define MINOTAUR_SENSOR_SETTINGS_HPP

#include <vector>
#include <string>

namespace minotaur
{
    /**
     * \brief The SensorSetting class contains all possible parameters
     *        for an ultrasonic sensor of the minotaur project.
     * 
     * It provides functions to load the parameters from the ROS
     * Paramserver. Therefore loading the parameters manually is not
     * needed anymore.
     */
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

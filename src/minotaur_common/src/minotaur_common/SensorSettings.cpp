#include <ros/ros.h>
#include "minotaur_common/SensorSettings.hpp"
#include "minotaur_common/MinotaurParam.hpp"

#define DEG_TO_RAD(deg) ((M_PI * deg) / 180)

namespace minotaur
{
    std::vector<SensorSetting> loadSensorSettings(const std::string &p_modelName)
    {
        if(!ros::master::check())
            throw std::logic_error("ROS master is not running. Cannot load sensor settings");
        
        std::vector<SensorSetting> result;    
        int i = 0;
        while(ros::param::has(PARAM_SENSOR(p_modelName, i)))
        {
            result.push_back(SensorSetting());
            
            result.back().id = i;
            
            // get direction and convert to radian
            ros::param::get(PARAM_SENSOR_DIRECTION(p_modelName, i), result.back().direction);
            result.back().direction = DEG_TO_RAD(result.back().direction);
            
            // get x and y positions relative to robot
            ros::param::get(PARAM_SENSOR_DX(p_modelName, i), result.back().x);
            ros::param::get(PARAM_SENSOR_DY(p_modelName, i), result.back().y);
            
            ++i;
        }
        
        return result;
    }
    
    std::vector<SensorSetting> loadCurrentSensorSettings()
    {
        if(!ros::master::check())
            throw std::logic_error("ROS master is not running. Cannot load sensor settings");

        std::string model;
        
        if(!ros::param::get(PARAM_CURRENT_MODEL(), model))
            throw std::logic_error("No current robot model available. Cannot load sensor settings");
            
        return loadSensorSettings(model);
    }
    
}

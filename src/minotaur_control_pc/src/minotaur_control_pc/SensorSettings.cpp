#include <ros/ros.h>
#include "minotaur_control_pc/SensorSettings.hpp"
#include "robot_control_beagle/Utils.hpp"

#define DEG_TO_RAD(deg) ((M_PI * deg) / 180)

namespace minotaur
{
    std::vector<SensorSetting> getSensorSettings()
    {
        if(!ros::master::check())
            throw std::logic_error("ROS master is not running. Cannot read sensor settings");
        
        std::vector<SensorSetting> result;
        std::string model;
        
        if(!ros::param::get(PARAM_CURRENT_MODEL(), model))
            throw std::logic_error("No current robot model available");
            
        int i = 0;
        while(ros::param::has(PARAM_SENSOR(model, i)))
        {
            result.push_back(SensorSetting());
            
            result.back().id = i;
            
            // get direction and convert to radian
            ros::param::get(PARAM_SENSOR_DIRECTION(model, i), result.back().direction);
            result.back().direction = DEG_TO_RAD(result.back().direction);
            
            // get x and y positions relative to robot
            ros::param::get(PARAM_SENSOR_DX(model, i), result.back().x);
            ros::param::get(PARAM_SENSOR_DY(model, i), result.back().y);
            
            ++i;
        }
        
        return result;
    }
}

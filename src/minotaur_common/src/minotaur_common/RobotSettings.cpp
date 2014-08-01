#include <ros/ros.h>
#include <stdexcept>
#include <sstream>
#include "minotaur_common/RobotSettings.hpp"
#include "minotaur_common/MinotaurParam.hpp"

#define DEF_MODEL_NAME "Unknown"

namespace minotaur
{
    RobotSettings::RobotSettings()
    :modelName(DEF_MODEL_NAME), pidParameter(), wheelTrack(1.f), wheelRadius(1.0f), samplingInterval(1) {  }
    RobotSettings::~RobotSettings() { }
    
    void RobotSettings::loadFromParamServer(const std::string &p_modelName)
    {
        if(!ros::master::check())
            throw std::logic_error("ROS master is not running. Cannot load RobotSettings");
        
        if(!ros::param::has(PARAM_MODEL(p_modelName))) {
            std::stringstream ss;
            ss << "Robot model \"" << p_modelName << "\" does not exist. Cannot load RobotSettings";
            throw std::logic_error(ss.str());
        }
        
        modelName = p_modelName;
        
        if(!ros::param::get(PARAM_WHEEL_TRACK(p_modelName), wheelTrack)) {
            std::stringstream ss;
            ss << "No wheeltrack found (" << p_modelName << "). Cannot load RobotSettings";
            throw std::logic_error(ss.str());
        }
        
        if(!ros::param::get(PARAM_WHEEL_RADIUS(p_modelName), wheelRadius)) {
            std::stringstream ss;
            ss << "No wheelradius found (" << p_modelName << "). Cannot load RobotSettings";
            throw std::logic_error(ss.str());
        }
        
        if(!ros::param::get(PARAM_KP(p_modelName), pidParameter.Kp)) {
            std::stringstream ss;
            ss << "No KP found (" << p_modelName << "). Cannot load RobotSettings";
            throw std::logic_error(ss.str());
        }
        
        if(!ros::param::get(PARAM_KI(p_modelName), pidParameter.Ki)) {
            std::stringstream ss;
            ss << "No KI found (" << p_modelName << "). Cannot load RobotSettings";
            throw std::logic_error(ss.str());
        }
        
        if(!ros::param::get(PARAM_KD(p_modelName), pidParameter.Kd)) {
            std::stringstream ss;
            ss << "No KD found (" << p_modelName << "). Cannot load RobotSettings";
            throw std::logic_error(ss.str());
        }
        
        if(!ros::param::get(PARAM_SAMPLING_INTERVAL(p_modelName), samplingInterval)) {
            std::stringstream ss;
            ss << "No sampling interval found (" << p_modelName << "). Cannot load RobotSettings";
            throw std::logic_error(ss.str());
        }
    }
    
    void RobotSettings::loadCurrentFromParamServer()
    {
        if(!ros::master::check())
            throw std::logic_error("ROS master is not running. Cannot load RobotSettings");
        
        std::string model;
        if(!ros::param::get(PARAM_CURRENT_MODEL(), model))
            throw std::logic_error("No current robot model available");
            
        loadFromParamServer(model);
    }
}

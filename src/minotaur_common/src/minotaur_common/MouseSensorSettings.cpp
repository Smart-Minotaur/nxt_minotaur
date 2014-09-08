#include <ros/ros.h>
#include "minotaur_common/MouseSensorSettings.hpp"
#include "minotaur_common/MinotaurParam.hpp"
#include "minotaur_common/Math.hpp"

namespace minotaur
{
	MouseSetting::MouseSetting()
	:id(-1), device("unkown"), x(0), y(0), errorAngle(0), xResolution(100), yResolution(100)
	{
		
	}
	
	MouseSetting::~MouseSetting()
	{
		
	}
	
	MouseSensorSettings::MouseSensorSettings()
	{
		
	}
	
	MouseSensorSettings::~MouseSensorSettings()
	{
		
	}
	
	MouseSetting& MouseSensorSettings::operator[](const int p_index)
	{
		return settings[p_index];
	}
	
	int MouseSensorSettings::size()
	{
		return settings.size();
	}
	
	void MouseSensorSettings::loadFromParamServer(const std::string &p_modelName)
	{
		settings.clear();
		if(!ros::master::check())
            throw std::logic_error("ROS master is not running. Cannot load MouseSensorSettings");
			
		if(!ros::param::has(PARAM_MODEL(p_modelName))) {
            std::stringstream ss;
            ss << "Robot model \"" << p_modelName << "\" does not exist. Cannot load MouseSensorSettings";
            throw std::logic_error(ss.str());
        }
		
		for(int i = 0; ros::param::has(PARAM_MOUSE(p_modelName, i)); ++i ) {
			
            settings.push_back(MouseSetting());
			settings[i].id = i;
			
			if(!ros::param::get(PARAM_MOUSE_DEVICE(p_modelName, i), settings[i].device)) {
				std::stringstream ss;
				ss << "No device found (" << p_modelName << "). Cannot load MouseSensorSettings";
				throw std::logic_error(ss.str());
			}
			
			if(!ros::param::get(PARAM_MOUSE_X(p_modelName, i), settings[i].x)) {
				std::stringstream ss;
				ss << "No x-offset found (" << p_modelName << "). Cannot load MouseSensorSettings";
				throw std::logic_error(ss.str());
			}
			
			if(!ros::param::get(PARAM_MOUSE_Y(p_modelName, i), settings[i].y)) {
				std::stringstream ss;
				ss << "No y-offset found (" << p_modelName << "). Cannot load MouseSensorSettings";
				throw std::logic_error(ss.str());
			}
			
			if(!ros::param::get(PARAM_MOUSE_ERROR_ANGLE(p_modelName, i), settings[i].errorAngle)) {
				std::stringstream ss;
				ss << "No errorAngle found (" << p_modelName << "). Cannot load MouseSensorSettings";
				throw std::logic_error(ss.str());
			}
			
			settings[i].errorAngle = normalizeRadian(degreeToRadian(settings[i].errorAngle));
			
			if(!ros::param::get(PARAM_MOUSE_X_RESOLUTION(p_modelName, i), settings[i].xResolution)) {
				std::stringstream ss;
				ss << "No X-Resolution found (" << p_modelName << "). Cannot load MouseSensorSettings";
				throw std::logic_error(ss.str());
			}
			
			if(!ros::param::get(PARAM_MOUSE_Y_RESOLUTION(p_modelName, i), settings[i].yResolution)) {
				std::stringstream ss;
				ss << "No Y-Resolution found (" << p_modelName << "). Cannot load MouseSensorSettings";
				throw std::logic_error(ss.str());
			}
        }
	}
	
	void MouseSensorSettings::loadCurrentFromParamServer()
	{
		if(!ros::master::check())
            throw std::logic_error("ROS master is not running. Cannot load MouseSensorSettings");
        
        std::string model;
        if(!ros::param::get(PARAM_CURRENT_MODEL(), model))
            throw std::logic_error("No current robot model available");
            
        loadFromParamServer(model);
	}
}
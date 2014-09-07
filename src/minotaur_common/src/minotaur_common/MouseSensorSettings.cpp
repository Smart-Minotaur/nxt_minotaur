#include <ros/ros.h>
#include "minotaur_common/MouseSensorSettings.hpp"
#include "minotaur_common/MinotaurParam.hpp"
#include "minotaur_common/Math.hpp"

namespace minotaur
{
	MouseSetting::MouseSetting()
	:dx(0), dy(0), errorAngle(0)
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
			
			if(!ros::param::get(PARAM_MOUSE_DX(p_modelName, i), settings[i].dx)) {
				std::stringstream ss;
				ss << "No DX found (" << p_modelName << "). Cannot load MouseSensorSettings";
				throw std::logic_error(ss.str());
			}
			
			if(!ros::param::get(PARAM_MOUSE_DY(p_modelName, i), settings[i].dy)) {
				std::stringstream ss;
				ss << "No DY found (" << p_modelName << "). Cannot load MouseSensorSettings";
				throw std::logic_error(ss.str());
			}
			
			if(!ros::param::get(PARAM_MOUSE_ERROR_ANGLE(p_modelName, i), settings[i].errorAngle)) {
				std::stringstream ss;
				ss << "No errorAngle found (" << p_modelName << "). Cannot load MouseSensorSettings";
				throw std::logic_error(ss.str());
			}
			
			settings[i].errorAngle = normalizeRadian(degreeToRadian(settings[i].errorAngle));
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
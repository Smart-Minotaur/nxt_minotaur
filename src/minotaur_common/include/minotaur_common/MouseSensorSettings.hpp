#ifndef MINOTAUR_MOUSE_SENSOR_SETTINGS_HPP
#define MINOTAUR_MOUSE_SENSOR_SETTINGS_HPP

namespace minotaur
{
	class MouseSetting
	{
	public:
		int id;
		std::string device;
		float x, y;
		float errorAngle;
		int xResolution, yResolution;
		
		MouseSetting();
		~MouseSetting();
	};
	
	class MouseSensorSettings
	{
	private:
		std::vector<MouseSetting> settings;
		
	public:
		MouseSensorSettings();
		~MouseSensorSettings();
		
		MouseSetting& operator[](const int p_index);
		int size();
		
		void loadFromParamServer(const std::string &p_modelName);
        void loadCurrentFromParamServer();
	};
}

#endif
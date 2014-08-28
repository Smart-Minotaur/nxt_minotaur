#ifndef NXT_INPUT_VALUES_HPP
#define NXT_INPUT_VALUES_HPP

#include <cstdint>

namespace nxt
{
	/**
	 * \brief Data structure for sensor measurements.
	 */
	class SensorInfo
	{
	public:
		uint16_t rawValue;
		uint16_t normalizedValue;
		int16_t scaledValue;
		int16_t calibratedValue;
	
		SensorInfo() { }
		~SensorInfo() { }
	};
	
	/**
	 * \brief Data structure for the current sensor state.
	 */
	class InputValues
	{
	public:
		uint8_t status;
		uint8_t port;
		uint8_t valid;
		uint8_t calibrated;
		uint8_t sensorType;
		uint8_t sensorMode;
		SensorInfo sensorInfo;
		
		InputValues() { }
		~InputValues() { }
	};
}

#endif
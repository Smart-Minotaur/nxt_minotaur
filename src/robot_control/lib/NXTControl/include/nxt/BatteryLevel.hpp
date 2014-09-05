#ifndef NXT_BATTERY_LEVEL_HPP
#define NXT_BATTERY_LEVEL_HPP

#include <cstdint>

namespace nxt
{
	/**
	 * \brief Data structure for GetBatteryLevel responses.
	 */
	class BatteryLevel
	{
	public:
		uint8_t status;
		uint16_t voltage;
		
		BatteryLevel() { }
		~BatteryLevel() { }
	};
}

#endif
#ifndef NXT_OUTPUT_STATE_HPP
#define NXT_OUTPUT_STATE_HPP

#include <cstdint>

namespace nxt
{
	/**
	 * \brief Data structure for tacho data that contains motor tick counts.
	 */
	class TachoInfo
	{
	public:
		uint32_t tachoLimit;
		int32_t tachoCount;
		int32_t blockTachoCount;
		int32_t rotationTachoCount;
	
		TachoInfo() { }
		~TachoInfo() { }
	};
	
	/**
	 * \brief Data structure for the current state of a motor.
	 */
	class OutputState
	{
	public:
		uint8_t status;
		uint8_t port;
		int8_t power;
		uint8_t motorMode;
		uint8_t regulationMode;
		int8_t turnRatio;
		uint8_t runState;
		TachoInfo tachoInfo;
	
		OutputState() { }
		~OutputState() { }
	};

}

#endif

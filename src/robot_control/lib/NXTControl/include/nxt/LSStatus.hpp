#ifndef NXT_LS_STATUS_HPP
#define NXT_LS_STATUS_HPP

#include <cstdint>

namespace nxt
{
	/**
	 * \brief Data structure for LSGetStatus responses.
	 */
	class LSStatus
	{
	public:
		uint8_t status;
		uint8_t readyBytesCount;
		
		LSStatus() { }
		~LSStatus() { }
	};
}

#endif
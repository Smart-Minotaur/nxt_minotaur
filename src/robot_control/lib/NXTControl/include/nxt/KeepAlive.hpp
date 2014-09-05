#ifndef NXT_KEEP_ALIVE_HPP
#define NXT_KEEP_ALIVE_HPP

#include <cstdint>

namespace nxt
{
	/**
	 * \brief Data structure for keep alive responses.
	 */
	class KeepAlive
	{
	public:
		uint8_t status;
		uint32_t sleepTimeLimit;
		
		KeepAlive() { }
		~KeepAlive() { }
	};
}

#endif
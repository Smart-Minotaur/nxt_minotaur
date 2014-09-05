#ifndef NXT_LS_READ_HPP
#define NXT_LS_READ_HPP

#include <cstdint>

namespace nxt
{
	/**
	 * \brief Data structure for LSRead responses.
	 */
	class LSRead
	{
	public:
		uint8_t status;
		uint8_t bytesRead;
		unsigned char data[16];
		
		LSRead() { }
		~LSRead() { }
	};
}

#endif

#ifndef NXT_PROGRAM_NAME_HPP
#define NXT_PROGRAM_NAME_HPP

#include <cstdint>
#include <string>

namespace nxt
{
	/**
	 * \brief Data structure for GetCurrentProgramName responses.
	 */
	class ProgramName
	{
	public:
		uint8_t status;
		std::string name;
		
		ProgramName() { }
		~ProgramName() { }
	};
}

#endif
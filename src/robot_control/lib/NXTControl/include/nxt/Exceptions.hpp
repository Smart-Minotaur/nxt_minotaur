#ifndef NXT_EXCEPTIONS_HPP
#define NXT_EXCEPTIONS_HPP

#include <stdexcept>

namespace nxt
{	
	/**
	 * \brief Standard exception in NXTControl.
	 */
	class NXTException: public std::logic_error
	{
	public:
		NXTException(const std::string& p_what) throw();
		NXTException(const char* p_what) throw();
	};
	
	/**
	 * \brief Exception for USB related problems.
	 */
	class USBException: public NXTException
	{
	public:
		USBException(const std::string& p_what) throw();
		USBException(const char* p_what) throw();
	};
	
	/**
	 * \brief Exception for Timeouts.
	 */
	class TimeoutException: public NXTException
	{
	public:
		TimeoutException(const std::string& p_what) throw();
		TimeoutException(const char* p_what) throw();
	};
}

#endif
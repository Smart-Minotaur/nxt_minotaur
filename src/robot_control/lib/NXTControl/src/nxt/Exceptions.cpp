#include "nxt/Exceptions.hpp"

namespace nxt
{
	NXTException::NXTException(const std::string& p_what) throw()
	:std::logic_error(p_what)
	{
		
	}
	
	NXTException::NXTException(const char* p_what) throw()
	:std::logic_error(std::string(p_what))
	{
	}
	
	USBException::USBException(const std::string& p_what) throw()
	:NXTException(p_what) 
	{
		
	}
	USBException::USBException(const char* p_what) throw()
	:NXTException(p_what)
	{
		
	}
	
	TimeoutException::TimeoutException(const std::string& p_what) throw()
	:NXTException(p_what)
	{
		
	}
	
	TimeoutException::TimeoutException(const char* p_what) throw()
	:NXTException(p_what)
	{
		
	}
}
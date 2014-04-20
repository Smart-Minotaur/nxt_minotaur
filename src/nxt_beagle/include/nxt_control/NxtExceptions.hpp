#ifndef NXT_CONTROL_NXT_EXCEPTIONS_HPP_
#define NXT_CONTROL_NXT_EXCEPTIONS_HPP_

#include <exception>
#include <stdexcept>
#include <string>
#include <sstream>

namespace nxtcon
{
    class USBDeviceNotFoundException : public std::logic_error
    {
    public:
        USBDeviceNotFoundException(std::string const& msg)
        :std::logic_error(msg) { }
    };
    
    class USBInterfaceNotFoundException : public std::logic_error
    {
    public:
        USBInterfaceNotFoundException(std::string const& msg)
        :std::logic_error(msg) { }
    };
    
    class USBBusyException : public std::logic_error
    {
    public:
        USBBusyException(std::string const& msg)
        :std::logic_error(msg) { }
    };
    
    class USBDeviceDisconnectException : public std::logic_error
    {
    public:
        USBDeviceDisconnectException(std::string const& msg)
        :std::logic_error(msg) { }
    };
    
    class USBPipeException : public std::logic_error
    {
    public:
        USBPipeException(std::string const& msg)
        :std::logic_error(msg) { }
    };
    
    class USBTimeoutException : public std::logic_error
    {
    public:
        USBTimeoutException(std::string const& msg)
        :std::logic_error(msg) { }
    };
    
    class USBError : public std::exception
    {
    private:
        std::string message;
        int code;
    public:
        USBError(std::string msg, const int p_code) noexcept
        {message = msg; code = p_code;}
        virtual ~USBError() noexcept { }
        
        const char* what() const noexcept
        {
            std::stringstream ss;
            ss << message << " Code: " << code;
            return ss.str().c_str();
        }
    };
    
    class NXTCommunicationException : public std::exception
    {
    private:
        std::string message;
        int code;
    public:
        NXTCommunicationException(std::string msg, const int p_code) noexcept
        {message = msg; code = p_code;}
        virtual ~NXTCommunicationException() noexcept { }
        
        const char* what() const noexcept
        { 
            std::stringstream ss;
            ss << message << " Code: " << code;
            return ss.str().c_str();
        }
    };
}

#endif
#ifndef NXT_CONTROL_NXT_EXCEPTIONS_HPP_
#define NXT_CONTROL_NXT_EXCEPTIONS_HPP_

#include <exception>
#include <string>
#include <sstream>

namespace nxtcon
{
    class USBException : public std::exception
    {
    private:
        std::string message;
    public:
        USBException(const char *msg) throw()
        :message(msg)
        { }
        
        USBException(const std::string &msg) throw()
        :message(msg)
        { }
        
        virtual ~USBException() throw() { }
        
        const char* what() const throw()
        {
            return message.c_str();
        }
    };
    
    class USBDeviceNotFoundException : public USBException
    {
    public:
        USBDeviceNotFoundException(std::string const& msg) throw()
        :USBException(msg) { }
    };
    
    class USBInterfaceNotFoundException : public USBException
    {
    public:
        USBInterfaceNotFoundException(std::string const& msg) throw()
        :USBException(msg) { }
    };
    
    class USBBusyException : public USBException
    {
    public:
        USBBusyException(std::string const& msg) throw()
        :USBException(msg) { }
    };
    
    class USBDeviceDisconnectException : public USBException
    {
    public:
        USBDeviceDisconnectException(std::string const& msg) throw()
        :USBException(msg) { }
    };
    
    class USBPipeException : public USBException
    {
    public:
        USBPipeException(std::string const& msg) throw()
        :USBException(msg) { }
    };
    
    class USBTimeoutException : public USBException
    {
    public:
        USBTimeoutException(std::string const& msg) throw()
        :USBException(msg) { }
    };
    
    class NXTException : public USBException
    {
    public:
        NXTException(std::string const& msg) throw()
        :USBException(msg) { }
    };
    
    class NXTTimeoutException : public NXTException
    {
    public:
        NXTTimeoutException(std::string const& msg) throw()
        :NXTException(msg) { }
    };
    
    class USBError : public std::exception
    {
    private:
        std::string message;
        int code;
    public:
        USBError(std::string msg, const int p_code) throw()
        {
            code = p_code;
            std::stringstream ss;
            ss << message << " Code: " << code;
            message = ss.str();
        }
        
        virtual ~USBError() throw() { }
        
        const char* what() const throw()
        {
            return message.c_str();
        }
        
        int getCode() const
        {
            return code;
        }
    };
    
    class NXTCommunicationException : public std::exception
    {
    private:
        std::string message;
        int code;
    public:
        
        NXTCommunicationException(const std::string& msg, const int p_code) throw()
        {
            code = p_code;
            std::stringstream ss;
            ss << message << " Code: " << code;
            message = ss.str();
        }
            
        virtual ~NXTCommunicationException() throw() { }
        
        const char* what() const throw()
        { 
            return message.c_str();
        }
        
        int getCode() const
        {
            return code;
        }
    };
}

#endif
#include "nxt_control/USBSocket.hpp"
#include "nxt_control/NxtExceptions.hpp"

#define DEF_BUF_SIZE 100

namespace minotaur
{
    USBSocket::USBSocket()
    {
        int ret = libusb_init(&libContext);
        if(ret < 0)
            throw USBError("Could not initialize libusb.", ret);
        
        wasAttached = false;
    }
    
    USBSocket::~USBSocket()
    {
        libusb_exit(libContext);
    }

    
    void USBSocket::open(const uint16_t p_vendorID, const uint16_t p_productID, const int p_interface)
    {
        int ret;
        interfaceNumber = p_interface;
        deviceHandle = libusb_open_device_with_vid_pid (libContext, p_vendorID, p_productID);
        if(deviceHandle == NULL)
            throw USBDeviceNotFoundException("Could not get USB device handle.");
        
        //check if interface is active / in use
        ret = libusb_kernel_driver_active(deviceHandle, interfaceNumber);
        if(ret)
        {
            wasAttached = true;
            libusb_detach_kernel_driver(deviceHandle, interfaceNumber);
        }
        
        //get interface
        ret = libusb_claim_interface(deviceHandle, interfaceNumber);
        if(ret) 
        {
            switch(ret)
            {
                case LIBUSB_ERROR_NOT_FOUND:
                    throw USBInterfaceNotFoundException("Could not claim USB interface.");
                    break;
                case LIBUSB_ERROR_BUSY:
                    throw USBBusyException("Could not claim USB interface.");
                    break;
                case LIBUSB_ERROR_NO_DEVICE:
                    USBDeviceNotFoundException("Could not claim USB interface.");
                    break;
            }
        }
    }
    
    void USBSocket::close()
    {
        if(wasAttached)
            libusb_attach_kernel_driver(deviceHandle, interfaceNumber);
        libusb_close(deviceHandle);
    }
    
    void USBSocket::send(const Telegram& p_telegram, unsigned char p_endpoint)
    {
        unsigned char data_array[p_telegram.getLength()];
        int transferredBytes, ret, sum = 0;
        p_telegram.getData(data_array);
        
        //transfer data in telegram, the endpoint gives the transfer direction
        //if endpoint wrong, transfer can be a receive instead of send
        ret = libusb_bulk_transfer(deviceHandle, p_endpoint, data_array, p_telegram.getLength(), &transferredBytes, 0);
        if(ret)
            throwTransferException(ret, "Could not send USB data.");
    }
    
    Telegram USBSocket::receive(unsigned char p_endpoint)
    {
        unsigned char data_array[DEF_BUF_SIZE];
        int transferredBytes, ret;
        Telegram result;
        
        //receive data from usb, the endpoint gives the transfer direction
        //if endpoint wrong, transfer can be a send instead of receive
        ret = libusb_bulk_transfer(deviceHandle, p_endpoint, data_array, DEF_BUF_SIZE, &transferredBytes, 0);
        if(ret)
            throwTransferException(ret, "Could not receive USB data.");
        
        result.addArray(data_array, transferredBytes);
        return result;
    }
    
    void USBSocket::throwTransferException(const int p_code, const std::string& p_msg)
    {
        switch(p_code)
        {
            case LIBUSB_ERROR_TIMEOUT:
                throw USBTimeoutException(p_msg);
                break;
            case LIBUSB_ERROR_PIPE:
                throw USBPipeException(p_msg);
                break;
            case LIBUSB_ERROR_NO_DEVICE:
                throw USBDeviceDisconnectException(p_msg);
                break;
            default:
                throw USBError(p_msg, p_code);
                break;
        }
    }
}
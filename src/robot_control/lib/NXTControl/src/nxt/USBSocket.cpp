#include "nxt/USBSocket.hpp"
#include "nxt/Exceptions.hpp"

#define MAX_NXT_MSG_SIZE 64

namespace nxt
{

	USBSocket::USBSocket()
	:wasAttached(false)
	{
        if(libusb_init(&libContext) < 0)
            throw USBException("Could not initialize libusb");
	}

	USBSocket::~USBSocket()
	{
		libusb_exit(libContext);
	}

	void USBSocket::open(const uint16_t p_vendorID, const uint16_t p_productID, const int p_interface)
	{
		int ret;
        interfaceNumber = p_interface;
        deviceHandle = libusb_open_device_with_vid_pid(libContext, p_vendorID, p_productID);
        if(deviceHandle == NULL)
            throw USBException("Could not get USB device handle");
        
        //check if interface is active / in use
        ret = libusb_kernel_driver_active(deviceHandle, interfaceNumber);
        if(ret) {
            wasAttached = true;
            libusb_detach_kernel_driver(deviceHandle, interfaceNumber);
        }
        
        //get interface
        ret = libusb_claim_interface(deviceHandle, interfaceNumber);
        if(ret) {
            switch(ret) {
                case LIBUSB_ERROR_NOT_FOUND:
                    throw USBException("Could not claim USB interface. Device not found");
                    break;
                case LIBUSB_ERROR_BUSY:
                    throw USBException("Could not claim USB interface. Device is busy");
                    break;
                case LIBUSB_ERROR_NO_DEVICE:
                    throw USBException("Could not claim USB interface. Device does not exist");
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
		unsigned char data_array[p_telegram.size()];
        int transferredBytes, ret;
        p_telegram.getData(data_array);
        
        //transfer data in telegram, the endpoint gives the transfer direction
        //if endpoint wrong, transfer can be a receive instead of send
        ret = libusb_bulk_transfer(deviceHandle, p_endpoint, data_array, p_telegram.size(), &transferredBytes, 0);
        if(ret)
            throw USBException("Could not send USB data");
	}
	
	Telegram USBSocket::receive(unsigned char p_endpoint)
	{
		unsigned char data_array[MAX_NXT_MSG_SIZE];
        int transferredBytes, ret;
        Telegram result;
        
        //receive data from usb, the endpoint gives the transfer direction
        //if endpoint is wrong, transfer can be a send instead of receive
        ret = libusb_bulk_transfer(deviceHandle, p_endpoint, data_array, MAX_NXT_MSG_SIZE, &transferredBytes, 0);
        if(ret)
            throw USBException("Could not receive USB data");
        
        result.addData(data_array, transferredBytes);
        return result;
	}
	
}


#ifndef NXT_USB_SOCKET_HPP
#define NXT_USB_SOCKET_HPP

#include <libusb-1.0/libusb.h>
#include "nxt/Telegram.hpp"

namespace nxt
{
	/**
	 * \brief Used to communicate with the LEGO NXT Brick via USB.
	 */
	class USBSocket
	{
	private:
		libusb_context *libContext;
        libusb_device_handle *deviceHandle;
		bool wasAttached;
        int interfaceNumber;
		
	public:
		USBSocket();
		~USBSocket();
		
		void open(const uint16_t p_vendorID, const uint16_t p_productID, const int p_interface);
        void close();
        
        void send(const Telegram& p_telegram, unsigned char p_endpoint);
        Telegram receive(unsigned char p_endpoint);
	};

}

#endif

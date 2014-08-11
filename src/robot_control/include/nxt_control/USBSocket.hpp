#ifndef NXT_CONTROL_USB_SOCKET_HPP_
#define NXT_CONTROL_USB_SOCKET_HPP_

#include <string>
#include "libusb-1.0/libusb.h"
#include "nxt_control/Telegram.hpp"

namespace nxtcon
{
    /**
     * \brief This class is used to communicate via USB
     * 
     * This class is used to find the Brick which has an USB-Port. 
     * If connection is established you can send Telegrams to the 
     * USB-Port to communicate with the Brick.
     */
    class USBSocket
    {
    private:
        libusb_context *libContext;
        libusb_device_handle *deviceHandle;
        bool wasAttached;
        int interfaceNumber;
        
        void throwTransferException(const int p_code, const std::string& p_msg);
    public:
        USBSocket();
        virtual ~USBSocket();
        
        void open(const uint16_t p_vendorID, const uint16_t p_productID, const int p_interface);
        void close();
        
        void send(const Telegram& p_telegram, unsigned char p_endpoint);
        Telegram receive(unsigned char p_endpoint);
    };
}

#endif
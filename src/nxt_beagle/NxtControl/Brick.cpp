#include "nxt_control/Brick.hpp"

namespace minotaur
{
    Brick::~Brick()
    {
        if(connected)
            usbSocket.close();
    }
    
    void Brick::find(const int p_interface = 0)
    {
        if(connected)
            throw std::logic_error("Cannot find Brick. Brick is already connected.");
        
        usbSocket.open(LEGO_VENDOR_ID, NXT_PRODUCT_ID, p_interface);
        connected = true;
    }
    
    bool Brick::isConnected() const
    {
        return connected;
    }
    
    usbSocket& Brick::getUSBSocket()
    {
        return usbSocket;
    }
}
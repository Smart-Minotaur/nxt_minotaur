#include <stdexcept>
#include "nxt_control/Brick.hpp"
#include "nxt_control/NxtOpcodes.hpp"

namespace nxtcon
{
    Brick::~Brick()
    {
        if(connected)
            usbSocket.close();
    }
    
    void Brick::find(const int p_interface)
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
    
    USBSocket& Brick::getUSBSocket()
    {
        return usbSocket;
    }
}
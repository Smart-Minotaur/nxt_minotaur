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
            throw std::logic_error("Cannot find Brick. Brick is already connected");
        
        usbSocket.open(LEGO_VENDOR_ID, NXT_PRODUCT_ID, p_interface);
        connected = true;
    }
    
    bool Brick::isConnected() const
    {
        return connected;
    }
    
    void Brick::send(const Telegram &p_telegram)
    {
        pthread_mutex_lock(&usbOutMutex);
        usbSocket.send(p_telegram, USB_OUT_ENDPOINT);
        pthread_mutex_unlock(&usbOutMutex);
    }
    
    Telegram Brick::sendWithResponse(const Telegram &p_telegram)
    {
        Telegram result;
        pthread_mutex_lock(&usbInMutex);
        pthread_mutex_lock(&usbOutMutex);
        
        usbSocket.send(p_telegram, USB_OUT_ENDPOINT);
        
        pthread_mutex_unlock(&usbOutMutex);
        
        result = usbSocket.receive(USB_IN_ENDPOINT);
        
        pthread_mutex_unlock(&usbInMutex);
        return result;
        
    }
}
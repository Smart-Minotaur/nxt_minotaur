#include <stdexcept>
#include <exception>
#include "nxt_control/Brick.hpp"
#include "nxt_control/NxtOpcodes.hpp"
#include "nxt_control/Lock.hpp"

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
        Lock lock(&usbOutMutex);
        usbSocket.send(p_telegram, USB_OUT_ENDPOINT);
    }
    
    Telegram Brick::sendWithResponse(const Telegram &p_telegram)
    {
        Telegram result;
        pthread_mutex_lock(&usbInMutex);
        pthread_mutex_lock(&usbOutMutex);
        
        try {
            usbSocket.send(p_telegram, USB_OUT_ENDPOINT);
        } catch (std::exception const &e) {
            pthread_mutex_unlock(&usbOutMutex);
            throw e;
        }
        pthread_mutex_unlock(&usbOutMutex);
        
        try {
            result = usbSocket.receive(USB_IN_ENDPOINT);
        } catch (std::exception const &e) {
            pthread_mutex_unlock(&usbInMutex);
            throw e;
        }
        pthread_mutex_unlock(&usbInMutex);
        
        return result;
    }
}
#ifndef NXT_CONTROL_BRICK_HPP_
#define NXT_CONTROL_BRICK_HPP_

#include <pthread.h>
#include "nxt_control/USBSocket.hpp"
#include "nxt_control/Telegram.hpp"

namespace nxtcon
{
    class Brick
    {
    private:
        USBSocket usbSocket;
        bool connected;
        pthread_mutex_t usbOutMutex;
        pthread_mutex_t usbInMutex;
    public:
        Brick()
        :usbSocket(), connected(false)
        {
            pthread_mutex_init(&usbOutMutex, NULL);
            pthread_mutex_init(&usbInMutex, NULL);
        }
        virtual  ~Brick();
        
        void find(const int p_interface = 0);
        bool isConnected() const;
        
        void send(const Telegram &p_telegram);
        Telegram receive();
    };
    
}

#endif
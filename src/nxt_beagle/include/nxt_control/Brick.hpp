#ifndef NXT_CONTROL_BRICK_HPP_
#define NXT_CONTROL_BRICK_HPP_

#include "nxt_control/USBSocket.hpp"

namespace minotaur
{
    class Brick
    {
    private:
        USBSocket usbSocket;
        bool connected;
    public:
        Brick()
        :usbSocket(), connected(false) { }
        virtual  ~Brick();
        
        void find();
        bool isConnected() const;
        usbSocket& getUSBSocket();
    };
    
}

#endif
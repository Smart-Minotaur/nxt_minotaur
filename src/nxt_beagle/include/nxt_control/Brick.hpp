#ifndef NXT_CONTROL_BRICK_HPP_
#define NXT_CONTROL_BRICK_HPP_

#include "nxt_control/USBSocket.hpp"

namespace nxtcon
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
        
        void find(const int p_interface = 0);
        bool isConnected() const;
        USBSocket& getUSBSocket();
    };
    
}

#endif
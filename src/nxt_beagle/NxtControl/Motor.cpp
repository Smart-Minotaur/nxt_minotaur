#include "nxt_control/Motor.hpp"
#include "nxt_control/NxtTelegram.hpp"

namespace nxtcon
{
    void Motor::setPower(const int8_t p_power)
    {
        Telegram telegram;
        //TODO if not working try with pointer cast
        create_setMotor(&telegram, port, (uint8_t) p_power);
        brick->getUSBSocket().send(telegram, USB_OUT_ENDPOINT);
    }
    
    void Motor::brake()
    {
        Telegram telegram;
        create_brakeMotor(&telegram, port);
        brick->getUSBSocket().send(telegram, USB_OUT_ENDPOINT);
    }
    
    void Motor::resetTacho()
    {
        Telegram telegram;
        create_resetTacho(&telegram, port);
        brick->getUSBSocket().send(telegram, USB_OUT_ENDPOINT);
    }
    
    const TachoData& Motor::getTachoData()
    {
        Telegram telegram;
        create_getOutputState(&telegram, port);
        brick->getUSBSocket().send(telegram, USB_OUT_ENDPOINT);
        telegram = brick->getUSBSocket().receive(USB_IN_ENDPOINT);
        decode_tachoOutputState(telegram, &tacho, port);
        return tacho;
    }
    
}
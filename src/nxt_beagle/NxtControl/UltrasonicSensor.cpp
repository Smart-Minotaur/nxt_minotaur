#include "nxt_control/UltrasonicSensor.hpp"
#include "nxt_control/NxtTelegram.hpp"
#include <stdexcept>

namespace nxtcon
{
    UltrasonicSensor::UltrasonicSensor(Brick *p_brick, const uint8_t p_port)
    :brick(p_brick),port(p_port), sensor()
    {
        if(!brick->isConnected())
            throw std::logic_error("Cannot create UltrasonicSensor. Brick is not connected or has not been searched.");
        
        Telegram telegram;
        create_setInputMode(&telegram, port, SENSOR_TYPE_LOWSPEED_9V, SENSOR_MODE_RAW);
        brick->getUSBSocket().send(telegram, USB_OUT_ENDPOINT);
    }
    const SensorData& UltrasonicSensor::getSensorData()
    {
        Telegram telegram;
        create_getInputValues(&telegram, port);
        brick->getUSBSocket().send(telegram, USB_OUT_ENDPOINT);
        telegram = brick->getUSBSocket().receive(USB_IN_ENDPOINT);
        decode_unltaSonicSensorInputValues(telegram, &sensor, port);
        return sensor;
    }
}
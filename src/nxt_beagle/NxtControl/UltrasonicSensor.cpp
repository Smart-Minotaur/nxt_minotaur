#include "nxt_control/UltrasonicSensor.hpp"
#include "nxt_control/NxtTelegram.hpp"

namespace nxtcon
{
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
#include "nxt/UltrasonicSensor.hpp"
#include "nxt/OPCodes.hpp"
#include "nxt/Exceptions.hpp"

#define MAX_TIMEOUT 30

namespace nxt
{

	UltrasonicSensor::UltrasonicSensor(Brick *p_brick, const uint8_t p_port)
	:brick(p_brick), port(p_port)
	{
		brick->setInputMode(port, SENSOR_TYPE_LOWSPEED_9V, SENSOR_MODE_RAW);
	}

	UltrasonicSensor::~UltrasonicSensor()
	{
		
	}
	
	uint8_t UltrasonicSensor::getDistance()
	{
		requestMeasurement();
		waitForReceive();
		LSRead result = brick->lsRead(port);
		// byte 4 contains distance
		return result.data[0];
	}
	
	void UltrasonicSensor::requestMeasurement()
	{
		unsigned char data[] = {LS_ULTRASONIC_ADDRESS, READ_ULTRASONIC_BYTE0};
		brick->lsWrite(port, 0x02, 0x01, data);
	}
	
	void UltrasonicSensor::waitForReceive()
	{
		int timeout = 0;
		
		while(1) {
			LSStatus status = brick->lsGetStatus(port);
			
			if(status.readyBytesCount != 0) {
				// received data
				break;
			}
			
			timeout++;
			if(timeout == MAX_TIMEOUT)
				measurementTimedOut();
		}
		
	}
	
	void UltrasonicSensor::measurementTimedOut() {
		// empty read for safety
		brick->lsRead(port);
		throw TimeoutException("Ultrasonic measurement timed out");
	}


}


#include "nxt/Motor.hpp"
#include "nxt/OPCodes.hpp"

namespace nxt
{

	Motor::Motor(Brick *p_brick, const uint8_t p_port)
	:brick(p_brick), port(p_port)
	{
	}

	Motor::~Motor()
	{
	}
	
	void Motor::setPower(const uint8_t p_power)
	{
		brick->setOutputState(port,
							 p_power,
							 MOTOR_MODE_ON_BRAKE,
							 REGULATION_MODE_NONE,
							 0,
							 RUN_STATE_RUNNING,
							 0);
	}
	
	void Motor::brake()
	{
		setPower(0);
	}
	
	void Motor::resetMotorPosition()
	{
		// reset motor with relative position
		brick->resetMotorPosition(port, 0x01);
	}
	
	TachoInfo Motor::getTachoInfo()
	{
		return brick->getOutputState(port).tachoInfo;
	}


}


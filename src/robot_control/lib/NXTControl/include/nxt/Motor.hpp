#ifndef NXT_MOTOR_HPP
#define NXT_MOTOR_HPP

#include "nxt/Brick.hpp"

namespace nxt
{
	/**
	* \brief Controls a motor of the brick.
	*/
	class Motor
	{
	private:
		Brick *brick;
		uint8_t port;
	public:
		Motor(Brick *p_brick, const uint8_t p_port);
		~Motor();
		
		void setPower(const uint8_t p_power);
		void brake();
		void resetMotorPosition();
		TachoInfo getTachoInfo();
	};

}

#endif

#include "mouse_monitor_pc/Robot.hpp"

namespace minotaur
{

	Robot::Robot() : attributes(RobotAttributes::getDefaultAttributes())
	{
		xPosition = 20.0;
		yPosition = 20.0;
		direction = 0.0;
	}

	Robot::Robot(RobotAttributes attributes) : attributes(attributes)
	{
	}

	Robot::~Robot()
	{
	}

	RobotAttributes Robot::getAttributes()
	{
		return attributes;
	}

	void Robot::setAttributes(RobotAttributes attributes)
	{
		this->attributes = attributes;
	}

	void Robot::setPosition(double x, double y)
	{
		xPosition = x;
		yPosition = y;
	}

	void Robot::move(double x, double y)
	{
		setPosition(xPosition + x, yPosition + y);
	}

	double Robot::xPos()
	{
		return xPosition;
	}

	double Robot::yPos()
	{
		return yPosition;
	}
	
	double Robot::getDirection()
	{
		return direction;
	}

}

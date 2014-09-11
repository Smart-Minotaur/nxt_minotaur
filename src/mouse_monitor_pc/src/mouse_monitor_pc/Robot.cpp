#include "mouse_monitor_pc/Robot.hpp"

#include <cmath>

namespace minotaur
{

	Robot::Robot() : attributes(RobotAttributes::getDefaultAttributes())
	{
		xPosition = 0.0;
		yPosition = 0.0;
		direction = M_PI/2.0;
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

	void Robot::forward(double delta)
	{
		xPosition += cos(direction) * delta;
		yPosition += sin(direction) * delta;
	}

	void Robot::rotate(double angle)
	{
		direction += angle;
	}

	void Robot::move(double dX, double dY)
	{
		double Vx_r = dX;
		double Vy_r = dX * attributes.m;
		double Vdist_r = sqrt(pow(Vx_r, 2) + pow(Vy_r, 2));
		double Vangle_r = atan(Vy_r/Vx_r);

		// From circular arc (Vdist_r)
		double rotateAngle = Vdist_r / attributes.distanceToSensor_radius;

		if (Vangle_r <= M_PI)
			rotate(rotateAngle);
		else
			rotate(rotateAngle * -1);

		double Vdist_f = dY - Vy_r;
		forward(Vdist_f);
	}

	void Robot::reset()
	{
		xPosition = 0.0;
		yPosition = 0.0;
		direction = M_PI/2.0;
	}

	double Robot::xPos()
	{
		return xPosition;
	}

	double Robot::yPos()
	{
		return yPosition;
	}

	double Robot::dir()
	{
		return direction;
	}

}

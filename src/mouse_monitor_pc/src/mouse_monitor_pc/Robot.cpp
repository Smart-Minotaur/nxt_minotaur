#include "mouse_monitor_pc/Robot.hpp"

#include <cmath>
#include <iostream>

namespace minotaur
{

	Robot::Robot() : attributes(RobotAttributes::getDefaultAttributes())
	{
		init();
	}

	Robot::Robot(RobotAttributes attributes) : attributes(attributes)
	{
		init();
	}

	Robot::~Robot()
	{
	}

	void Robot::init()
	{
		xPosition = 0.0;
		yPosition = 0.0;
		direction = M_PI/2.0;

		sensor1.set(attributes.distanceToSensor_x, attributes.distanceToSensor_y, direction);
		sensor2.set(-attributes.distanceToSensor_x, attributes.distanceToSensor_y, direction);
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
		double Vy_r = dX * attributes.m; // TODO: Problem: m ist not exact enough!
		
		double Vdist_r = sqrt(pow(Vx_r, 2) + pow(Vy_r, 2));

		double Vm_r = Vy_r/Vx_r;
		double Vangle_r = atan(Vm_r);

		// From circular arc (Vdist_r)
		double rotateAngle = Vdist_r / attributes.distanceToSensor_radius;

/*		std::cout << "ANGLE: " << dX / dY << std::endl;
		std::cout << "angle: " << Vangle_r << std::endl;
		std::cout << "angle attr.: " << attributes.sensorAngle << std::endl;
		std::cout << "m attr.: " << attributes.m << std::endl;*/

		if (Vx_r >= 0)
			rotate(rotateAngle);
		else
			rotate(rotateAngle * -1);

		//double Vdist_f = dY - Vy_r;
		double Vdist_f = sqrt(pow(dX, 2) + pow(dY, 2)) - Vdist_r;
		std::cout << "Forward: " << Vdist_f << std::endl;
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

	Sensor &Robot::s1()
	{
		return sensor1;
	}

	Sensor &Robot::s2()
	{
		return sensor2;
	}

}

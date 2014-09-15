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
		pos.xPosition = 0.0;
		pos.yPosition = 0.0;
		direction = M_PI/2.0;

		sensor1.set(pos.xPosition + attributes.distanceToSensor_x, pos.yPosition + attributes.distanceToSensor_y, direction);
		sensor2.set(pos.xPosition + -attributes.distanceToSensor_x, pos.yPosition + attributes.distanceToSensor_y, direction);
	}

	RobotAttributes Robot::getAttributes()
	{
		return attributes;
	}

	void Robot::setAttributes(RobotAttributes attributes)
	{
		this->attributes = attributes;
	}

	void Robot::forward(double delta)
	{
		Object::forward(delta);
	}

	void Robot::rotate(double angle)
	{
		Object::rotate(angle);

		sensor1.rotate(angle);
		sensor2.rotate(angle);

		double s1_xNew = (std::cos(angle) * sensor1.xPos()) + (-std::sin(angle) * sensor1.yPos());
		double s1_yNew = (std::sin(angle) * sensor1.xPos()) + (std::cos(angle) * sensor1.yPos());
		sensor1.set(s1_xNew, s1_yNew, sensor1.dir());

		double s2_xNew = (std::cos(angle) * sensor2.xPos()) + (-std::sin(angle) * sensor2.yPos());
		double s2_yNew = (std::sin(angle) * sensor2.xPos()) + (std::cos(angle) * sensor2.yPos());
		sensor2.set(s2_xNew, s2_yNew, sensor2.dir());
	}

	void Robot::move(double dX, double dY)
	{
		double Vx_r = dX;
		double Vy_r = dX * attributes.m;

		double Vdist_r = sqrt(pow(Vx_r, 2) + pow(Vy_r, 2));

		double Vm_r = Vy_r/Vx_r;
		double Vangle_r = atan(Vm_r);

		// From circular arc (Vdist_r)
		double rotateAngle = Vdist_r / attributes.distanceToSensor_radius;
		
		// Rotation direction
		if (dX >= 0)
			rotate(rotateAngle);
		else
			rotate(rotateAngle * -1);
			
		double Vdist_rf = sqrt(pow(dX, 2) + pow(dY, 2));
		double Vdist_f = Vdist_rf - Vdist_r;		
		
		// Forward or backward
		if (dY < 0)
			Vdist_f *= -1;

		forward(Vdist_f);
	}

	void Robot::reset()
	{
		pos.xPosition = 0.0;
		pos.yPosition = 0.0;
		direction = M_PI/2.0;

		sensor1.set(pos.xPosition + attributes.distanceToSensor_x, pos.yPosition + attributes.distanceToSensor_y, direction);
		sensor2.set(pos.xPosition + -attributes.distanceToSensor_x, pos.yPosition + attributes.distanceToSensor_y, direction);
		
		Object::reset();

		sensor1.reset();
		sensor2.reset();
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

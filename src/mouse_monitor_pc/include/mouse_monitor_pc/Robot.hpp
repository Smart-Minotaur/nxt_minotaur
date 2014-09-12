#ifndef ROBOT_HPP
#define ROBOT_HPP

#include <cmath>

namespace minotaur
{

	struct RobotAttributes {
		double axisLength; // cm

		double distanceToSensor_x; // cm
		double distanceToSensor_y; // cm

		double distanceToSensor_radius; // cm
		double distanceToWheel; // cm
		double sensorAngle; // Radiants
		double m;

		RobotAttributes(double axis,
		                double dx,
		                double dy) :
			axisLength(axis),
			distanceToSensor_x(dx),
			distanceToSensor_y(dy) {

			// TODO: Get correct robot length data
			sensorAngle = (M_PI/2.0) - atan(distanceToSensor_y/distanceToSensor_x);
			m = tan(sensorAngle);

			distanceToSensor_radius = sqrt(pow(distanceToSensor_x, 2) + pow(distanceToSensor_y, 2));
			distanceToWheel = axisLength / 2.0;
		}

		static RobotAttributes getDefaultAttributes() {
			RobotAttributes attributes(7.9, 3.95, 4.9);
			return attributes;
		}
	};

	class Sensor
	{
		public:
			// Position in the robot coordinate system
			double xPosition;
			double yPosition;

			double direction;

			void set(double dx, double dy, double dir) {
				xPosition = dx;
				yPosition = dy;
				direction = dir;
			}

			double xPos() {
				return xPosition;
			}

			double yPos() {
				return yPosition;
			}

			double dir() {
				return direction;
			}
	};

	class Robot
	{
		private:
			RobotAttributes attributes;

			// Position in the global coordinate system
			double xPosition;
			double yPosition;

			/**
			 * The direction is the angle y-axis of the robot
			 * in radiant.
			 */
			double direction;

			Sensor sensor1;
			Sensor sensor2;

			void init();

		public:
			Robot();
			Robot(RobotAttributes attributes);
			~Robot();

			void setAttributes(RobotAttributes attributes);
			RobotAttributes getAttributes();

			void setPosition(double x, double y);
			void forward(double delta);
			void rotate(double angle);
			void move(double dX, double dY);
			void reset();

			Sensor &s1();
			Sensor &s2();

			double xPos();
			double yPos();
			double dir();
	};

}

#endif

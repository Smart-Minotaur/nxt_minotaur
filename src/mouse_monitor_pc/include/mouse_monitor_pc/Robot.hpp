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
				
			sensorAngle = (M_PI/180) * 45 ; // TODO
			m = atan(sensorAngle);
			
			distanceToSensor_radius = sqrt(pow(distanceToSensor_x, 2) + pow(distanceToSensor_y, 2)); 
			distanceToWheel = axisLength / 2.0;
		}

		static RobotAttributes getDefaultAttributes() {
			RobotAttributes attributes(7.9, 3.95, 4.9);
			return attributes;
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

			double xPos();
			double yPos();
			double dir();
	};

}

#endif

#ifndef ROBOT_HPP
#define ROBOT_HPP

namespace minotaur
{

	struct RobotAttributes {
		double axisLength; // cm
		double distanceToWheel; // cm
		double distanceToSensor_x; // cm
		double distanceToSensor_y; // cm
		double distanceToSensor_radius; // cm

		RobotAttributes(double axis,
		                double dx,
		                double dy,
		                double dr) :
			axisLength(axis),
			distanceToSensor_x(dx),
			distanceToSensor_y(dy),
			distanceToSensor_radius(dr) {
			distanceToWheel = axisLength / 2.0;
		}

		static RobotAttributes getDefaultAttributes() {
			RobotAttributes attributes(4.0, 1.0, 1.5, 0.0);
			return attributes;
		}
	};

	class Robot
	{
		private:
			RobotAttributes attributes;

			double xPosition;
			double yPosition;
			double direction;

		public:
			Robot();
			Robot(RobotAttributes attributes);
			~Robot();

			void setAttributes(RobotAttributes attributes);
			RobotAttributes getAttributes();
			
			void setPosition(double x, double y);
			void move(double x, double y);
			double xPos();
			double yPos();
			double getDirection();
	};

}

#endif

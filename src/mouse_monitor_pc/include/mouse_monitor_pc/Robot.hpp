#ifndef ROBOT_HPP
#define ROBOT_HPP

#include <cmath>
#include <vector>

namespace minotaur
{

	struct Position {
		double xPosition;
		double yPosition;
	};

	class Object
	{
		protected:
			Position pos;

			/**
			 * The direction is the angle y-axis of the object
			 * in radiant.
			 */
			double direction;

			/**
			 * For path tracking (in global coordinate system).
			 */
			std::vector<Position> path;

		public:
			virtual void set(double dx, double dy, double dir) {
				pos.xPosition = dx;
				pos.yPosition = dy;
				direction = dir;
			}

			virtual void forward(double delta) {
				pos.xPosition += cos(direction) * delta;
				pos.yPosition += sin(direction) * delta;
			}

			virtual void rotate(double angle) {
				direction += angle;
			}

			virtual double xPos() {
				return pos.xPosition;
			}

			virtual double yPos() {
				return pos.yPosition;
			}

			virtual double dir() {
				return direction;
			}

			virtual void reset() {
				path.clear();
			}

			virtual std::vector<Position> &getPath() {
				return path;
			}

			virtual void addPath(Position pos) {
				path.push_back(pos);
			}
	};

	class Sensor : public Object
	{
		public:
			virtual void rotate(double angle) {
				Object::rotate(angle);

				double xNew = (std::cos(angle) * pos.xPosition) + (-std::sin(angle) * pos.yPosition);
				double yNew = (std::sin(angle) * pos.xPosition) + (std::cos(angle) * pos.yPosition);

				pos.xPosition = xNew;
				pos.yPosition = yNew;
			}
	};

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

			sensorAngle = (M_PI/2.0) - std::atan2(distanceToSensor_y, distanceToSensor_x);
			m = tan(sensorAngle);

			distanceToSensor_radius = sqrt(pow(distanceToSensor_x, 2) + pow(distanceToSensor_y, 2));
			distanceToWheel = axisLength / 2.0;
		}

		static RobotAttributes getDefaultAttributes() {
			RobotAttributes attributes(7.9, 3.95, 4.9);
			return attributes;
		}
	};

	class Robot : public Object
	{
		private:
			RobotAttributes attributes;

			Sensor sensor1; // Position relative to robot coordinate system
			Sensor sensor2; // Position relative to robot coordinate system

			void init();

		public:
			Robot();
			Robot(RobotAttributes attributes);
			~Robot();

			void setAttributes(RobotAttributes attributes);
			RobotAttributes getAttributes();

			virtual void forward(double delta);
			virtual void rotate(double angle);

			void move(double dX, double dY);
			void reset();

			Sensor &s1();
			Sensor &s2();
	};

}

#endif

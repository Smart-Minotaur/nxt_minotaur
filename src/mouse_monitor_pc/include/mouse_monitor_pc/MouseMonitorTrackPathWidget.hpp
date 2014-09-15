#ifndef MOUSE_MONITOR_TRACK_PATH_WIDGET_H
#define MOUSE_MONITOR_TRACK_PATH_WIDGET_H

#include <QWidget>
#include <QPainterPath>
#include <QColor>

#include "mouse_monitor_pc/MouseMonitorNode.hpp"
#include "mouse_monitor_pc/Robot.hpp"

namespace minotaur
{

	struct coordinateSystem {
		double originalCenterX;
		double originalCenterY;

		double centerX;
		double centerY;

		void setCenter(double x, double y) {
			originalCenterX = x;
			originalCenterY = y;

			centerX = x;
			centerY = y;
		}

		double up(double d) {
			return (centerY - d);
		}

		double down(double d) {
			return (centerY + d);
		}

		double left(double d) {
			return (centerX - d);
		}

		double right(double d) {
			return (centerX + d);
		}

		static double upFrom(double from, double d) {
			return (from - d);
		}

		static double downFrom(double from, double d) {
			return (from + d);
		}

		static double leftFrom(double from, double d) {
			return (from - d);
		}

		static double rightFrom(double from, double d) {
			return (from + d);
		}

		void scale(int zoom) {
			centerX = originalCenterX / zoom;
			centerY = originalCenterY / zoom;
		}
	};

	class TrackPathWidget : public QWidget
	{
			Q_OBJECT

		private:
			coordinateSystem globalCoordinateSystem;

			QPointF lastMousePos;

			double translatex;
			double translatey;

			int zoom;
			bool sensor1_enable;
			bool sensor2_enable;

			Robot robot;

			void drawGrid(QPainter &painter);
			void drawRobot(QPainter &painter);
			void drawGlobalCoordinateSystem(QPainter &painter);
			void translateAndScale(QPainter &painter);
			void drawCoordinateSystem(double x, double y, double dir,
			                          QColor xColor, QColor yColor, double len, double width,
			                          QPainter &painter);
			void drawPath(std::vector<Position> &path, QColor color, QPainter &painter);
			void drawSensorPath(std::vector<Position> &path, QColor color, QPainter &painter);
			void drawRobotAxis(double globalX, double globalY, QPainter &painter);
			void rotateObject(double &x, double &y, double angle);

		protected:
			void paintEvent(QPaintEvent *event);
			void mouseMoveEvent(QMouseEvent *event);
			void mouseReleaseEvent(QMouseEvent *event);

		public:
			TrackPathWidget(QWidget *parent = 0) : QWidget(parent) {}
			virtual ~TrackPathWidget() {}

			void init();
			void reset();

			void updateRobot(Robot robot);

		public Q_SLOTS:
			void zoomValueChanged(const int value);
			void sensor1Enable(const int status);
			void sensor2Enable(const int status);
	};

}

#endif

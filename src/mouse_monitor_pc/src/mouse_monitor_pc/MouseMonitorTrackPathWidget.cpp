#include "mouse_monitor_pc/MouseMonitorTrackPathWidget.hpp"

#include <QPainter>
#include <QToolTip>
#include <QMouseEvent>

#include <math.h>

// Direction grid params
#define GRID_SCALE 10
#define GLOBAL_COORD_DIRECTION_LEN 5
#define ROBOT_COORD_DIRECTION_LEN 5

namespace minotaur
{

	void TrackPathWidget::init()
	{
		translatex = 0.0;
		translatey = 0.0;
		lastMousePos.setX(0.0);
		lastMousePos.setY(0.0);

		zoom = 1;
		sensor1_enable = true;
		sensor2_enable = true;
	}

	void TrackPathWidget::paintEvent(QPaintEvent *event)
	{
		globalCoordinateSystem.setCenter(width() / 2.0, height() / 2.0);

		QPainter painter(this);
		painter.setRenderHint(QPainter::Antialiasing);

		translateAndScale(painter);

		drawGrid(painter);
		drawGlobalCoordinateSystem(painter);
		drawRobot(painter);
		drawPath(robot.getPath(), Qt::darkRed, painter);
		if (sensor1_enable)
			drawSensorPath(robot.s1().getPath(), Qt::darkGreen, painter);
		if (sensor2_enable)
			drawSensorPath(robot.s2().getPath(), Qt::darkBlue, painter);
	}

	void TrackPathWidget::drawGrid(QPainter &painter)
	{
		double gridWidth = this->width();
		double gridHeight = this->height();

		int xSteps = (int) (gridWidth / GRID_SCALE);
		int ySteps = (int) (gridHeight / GRID_SCALE);

		double fromX = globalCoordinateSystem.centerX - xSteps/2 * GRID_SCALE;
		double fromY = globalCoordinateSystem.centerY - ySteps/2 * GRID_SCALE;

		painter.setPen(QPen(Qt::gray, 0.5));

		for (int i = 0; i <= xSteps; ++i) {
			QLineF line(fromX + i * GRID_SCALE, fromY,
			            fromX + i * GRID_SCALE, fromY + ySteps * GRID_SCALE);
			painter.drawLine(line);
		}

		for (int i = 0; i <= ySteps; ++i) {
			QLineF line(fromX, fromY + i * GRID_SCALE,
			            fromX + xSteps * GRID_SCALE, fromY + i * GRID_SCALE);
			painter.drawLine(line);
		}
	}

	void TrackPathWidget::drawRobot(QPainter &painter)
	{
		double globalX = globalCoordinateSystem.right(robot.xPos());
		double globalY = globalCoordinateSystem.up(robot.yPos());

		double globalX_s1 = globalCoordinateSystem.right(robot.xPos() + robot.s1().xPos());
		double globalY_s1 = globalCoordinateSystem.up(robot.yPos() + robot.s1().yPos());

		double globalX_s2 = globalCoordinateSystem.right(robot.xPos() + robot.s2().xPos());
		double globalY_s2 = globalCoordinateSystem.up(robot.yPos() + robot.s2().yPos());

		// TODO: drawRobotAxis(globalX, globalY, painter);

		// Draw robot coordinate system
		drawCoordinateSystem(globalX,
		                     globalY,
		                     robot.dir(), Qt::blue, Qt::red, ROBOT_COORD_DIRECTION_LEN, 0.3,
		                     painter);

		// Draw sensor 1 coordinate system
		drawCoordinateSystem(globalX_s1,
		                     globalY_s1,
		                     robot.s1().dir(), Qt::magenta, Qt::cyan, ROBOT_COORD_DIRECTION_LEN, 0.1,
		                     painter);

		// Draw sensor 2 coordinate system
		drawCoordinateSystem(globalX_s2,
		                     globalY_s2,
		                     robot.s2().dir(), Qt::magenta, Qt::cyan, ROBOT_COORD_DIRECTION_LEN, 0.1,
		                     painter);
	}

	void TrackPathWidget::rotateObject(double &x, double &y, double angle)
	{
		double x1New = (std::cos(angle) * x) + (-std::sin(angle) * y);
		double y1New = (std::sin(angle) * x) + (std::cos(angle) * y);
	}

	void TrackPathWidget::drawRobotAxis(double globalX, double globalY, QPainter &painter)
	{
		double axisHeight = 0.5;
		double x1, y1, x2, y2;

		painter.setPen(QPen(Qt::black, axisHeight));

		x1 = globalX - robot.getAttributes().distanceToWheel;
		y1 = globalY;

		x2 = globalX + robot.getAttributes().distanceToWheel;
		y2 = globalY;

		QLineF axis(x1, y1, x2, y2);
		axis.setAngle((180/M_PI) * (robot.dir() - M_PI/2.0));
		painter.drawLine(axis);

		painter.setPen(QPen(Qt::green, axisHeight/2.0));

		x1 = globalX - robot.getAttributes().distanceToWheel - axisHeight/2.0;
		y1 = globalY - axisHeight;
		x2 = axisHeight;
		y2 = axisHeight*2;

		QRectF wheel1(x1, y1, x2, y2);

		x1 = globalX + robot.getAttributes().distanceToWheel - axisHeight/2.0;
		y1 = globalY - axisHeight;
		x2 = axisHeight;
		y2 = axisHeight*2;

		QRectF wheel2(x1, y1, x2, y2);
		//wheel1.setAngle((180/M_PI) * (robot.dir() - M_PI/2.0));
		//wheel2.setAngle((180/M_PI) * (robot.dir() - M_PI/2.0));

		//painter.rotate((180/M_PI) * (robot.dir() - M_PI/2.0));

		//painter.drawRect(wheel1);
		//painter.drawRect(wheel2);

		//painter.rotate(0);
	}

	void TrackPathWidget::drawGlobalCoordinateSystem(QPainter &painter)
	{
		drawCoordinateSystem(globalCoordinateSystem.centerX,
		                     globalCoordinateSystem.centerY,
		                     M_PI/2.0, Qt::blue, Qt::red, GLOBAL_COORD_DIRECTION_LEN, 0.5,
		                     painter);
	}

	void TrackPathWidget::drawPath(std::vector<Position> &path, QColor color, QPainter &painter)
	{
		painter.setPen(QPen(color, 0.5));

		for (std::vector<Position>::iterator i = path.begin(); i != path.end(); ++i) {
			QPointF point(globalCoordinateSystem.right(i->xPosition), globalCoordinateSystem.up(i->yPosition));
			painter.drawPoint(point);
		}
	}

	void TrackPathWidget::drawSensorPath(std::vector<Position> &path, QColor color, QPainter &painter)
	{
		painter.setPen(QPen(color, 0.5));

		for (std::vector<Position>::iterator i = path.begin(); i != path.end(); ++i) {
			QPointF point(globalCoordinateSystem.right(robot.xPos() + i->xPosition),
			              globalCoordinateSystem.up(robot.yPos() + i->yPosition));
			painter.drawPoint(point);
		}
	}

	void TrackPathWidget::drawCoordinateSystem(
	    double x,
	    double y,
	    double dir,
	    QColor xColor,
	    QColor yColor,
	    double len,
	    double width,
	    QPainter &painter)
	{
		QLineF lineY(x,
		             y,
		             coordinateSystem::rightFrom(x, cos(dir) * len),
		             coordinateSystem::upFrom(y, sin(dir) * len));
		painter.setPen(QPen(yColor, width));
		painter.drawLine(lineY);

		QLineF lineX(x,
		             y,
		             coordinateSystem::rightFrom(x, cos(dir - M_PI/2.0) * len),
		             coordinateSystem::upFrom(y, sin(dir - M_PI/2.0) * len));
		painter.setPen(QPen(xColor, width));
		painter.drawLine(lineX);
	}

	void TrackPathWidget::translateAndScale(QPainter &painter)
	{
		painter.setRenderHint(QPainter::Antialiasing);
		painter.translate(QPointF(translatex, translatey));

		painter.scale((qreal) zoom, (qreal) zoom);
		globalCoordinateSystem.scale(zoom);
	}

	void TrackPathWidget::zoomValueChanged(const int value)
	{
		zoom = value;
		update();
	}

	void TrackPathWidget::sensor1Enable(const int status)
	{
		if (status == 2)
			this->sensor1_enable = true;
		else
			this->sensor1_enable = false;

		update();
	}

	void TrackPathWidget::sensor2Enable(const int status)
	{
		if (status == 2)
			this->sensor2_enable = true;
		else
			this->sensor2_enable = false;

		update();
	}

	void TrackPathWidget::mouseMoveEvent(QMouseEvent *event)
	{
		QString str = "Move mouse to translate map";
		QToolTip::showText(this->mapToGlobal(event->pos()), str, this);

		if (lastMousePos.x() == 0 && lastMousePos.y() == 0) {
			lastMousePos = event->posF();
			return;
		}

		translatex += event->x() - lastMousePos.x();
		translatey += event->y() - lastMousePos.y();

		lastMousePos = event->posF();

		update();
	}

	void TrackPathWidget::mouseReleaseEvent(QMouseEvent *event)
	{
		lastMousePos.setX(0.0);
		lastMousePos.setY(0.0);
	}

	void TrackPathWidget::reset()
	{
		translatex = 0.0;
		translatey = 0.0;
		lastMousePos.setX(0.0);
		lastMousePos.setY(0.0);

		robot.reset();

		update();
	}

	void TrackPathWidget::updateRobot(Robot robot)
	{
		this->robot = robot;
		update();
	}

}

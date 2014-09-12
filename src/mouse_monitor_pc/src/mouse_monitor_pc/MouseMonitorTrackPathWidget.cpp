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
		//startx = this->width() / 2.0; //posx;
		//starty = this->height() / 2.0; //posy;
		sensor1_path.moveTo(QPointF(globalCoordinateSystem.centerX, globalCoordinateSystem.centerY));
		sensor2_path.moveTo(QPointF(globalCoordinateSystem.centerX, globalCoordinateSystem.centerY));

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

		translateAndScale(painter);

		drawGrid(painter);
		drawGlobalCoordinateSystem(painter);
		drawRobot(painter);

		// Draw path
		if (sensor1_enable) {
			sensor1_path.translate(QPointF(globalCoordinateSystem.centerX, globalCoordinateSystem.centerY));
			painter.setPen(Qt::blue);
			painter.drawPath(sensor1_path);
		}

		if (sensor2_enable) {
			sensor1_path.translate(QPointF(globalCoordinateSystem.centerX, globalCoordinateSystem.centerY));
			painter.setPen(Qt::red);
			painter.drawPath(sensor2_path);
		}
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
		double axisHeight = 0.5;

		double globalX = globalCoordinateSystem.right(robot.xPos());
		double globalY = globalCoordinateSystem.up(robot.yPos());

		/*
				painter.setPen(QPen(Qt::black, axisHeight));
				QLineF axis(globalX - robot.getAttributes().distanceToWheel, globalY,
				            globalX + robot.getAttributes().distanceToWheel, globalY);
				painter.drawLine(axis);

				painter.setPen(QPen(Qt::green, axisHeight/2.0));
				QRectF wheel1(globalX - robot.getAttributes().distanceToWheel - axisHeight/2.0,
				              globalY - axisHeight, axisHeight, axisHeight*2);
				QRectF wheel2(globalX + robot.getAttributes().distanceToWheel - axisHeight/2.0,
				              globalY - axisHeight, axisHeight, axisHeight*2);

				painter.drawRect(wheel1);
				painter.drawRect(wheel2);*/

		// Draw coordinate system
		/*double len = ROBOT_COORD_DIRECTION_LEN;

		painter.setPen(QPen(Qt::red, 0.5));
		QLineF lineY(globalX,
		             globalY,
		             coordinateSystem::rightFrom(globalX, cos(robot.dir()) * len),
		             coordinateSystem::upFrom(globalY, sin(robot.dir()) * len));
		painter.drawLine(lineY);

		painter.setPen(QPen(Qt::blue, 0.5));
		QLineF lineX(globalX,
		             globalY,
		             coordinateSystem::rightFrom(globalX, cos((robot.dir() - M_PI/2.0)) * len),
		             coordinateSystem::upFrom(globalY, sin((robot.dir() - M_PI/2.0)) * len));
		painter.drawLine(lineX);*/
		
		
	}

	void TrackPathWidget::drawGlobalCoordinateSystem(QPainter &painter)
	{
		drawCoordinateSystem(globalCoordinateSystem.centerX,
		                     globalCoordinateSystem.centerY,
		                     M_PI/2.0, Qt::blue, Qt::red, GLOBAL_COORD_DIRECTION_LEN,
							 painter);

		/*painter.setPen(QPen(Qt::red, 0.5));
		QLineF lineY(globalCoordinateSystem.centerX,
		             globalCoordinateSystem.centerY,
		             globalCoordinateSystem.centerX,
		             globalCoordinateSystem.up(GLOBAL_COORD_DIRECTION_LEN));
		painter.drawLine(lineY);

		QLineF lineX(globalCoordinateSystem.centerX,
		             globalCoordinateSystem.centerY,
		             globalCoordinateSystem.right(GLOBAL_COORD_DIRECTION_LEN),
		             globalCoordinateSystem.centerY);
		painter.setPen(QPen(Qt::blue, 0.5));
		painter.drawLine(lineX);*/
	}

	void TrackPathWidget::translateAndScale(QPainter &painter)
	{
		painter.setRenderHint(QPainter::Antialiasing);
		painter.translate(QPointF(translatex, translatey));

		painter.scale((qreal) zoom, (qreal) zoom);
		globalCoordinateSystem.scale(zoom);
	}

	void TrackPathWidget::drawCoordinateSystem(
	    double x,
	    double y,
	    double dir,
	    QColor xColor,
	    QColor yColor,
		double len,
	    QPainter &painter)
	{
		QLineF lineY(x,
		             y,
		             coordinateSystem::rightFrom(x, cos(dir) * len),
		             coordinateSystem::upFrom(y, sin(dir) * len));
		painter.setPen(QPen(yColor, 0.5));
		painter.drawLine(lineY);

		QLineF lineX(x,
		             y,
		             coordinateSystem::rightFrom(x, cos(dir - M_PI/2.0) * len),
		             coordinateSystem::upFrom(y, sin(dir - M_PI/2.0) * len));
		painter.setPen(QPen(xColor, 0.5));
		painter.drawLine(lineX);
	}

	// TODO: Remove that and make it in update robot
	void TrackPathWidget::updateWidget(MouseData data)
	{
		if (data.id == "/dev/spidev1.0")
			sensor1_path.lineTo(sensor1_path.currentPosition() + QPointF(data.x_disp, data.y_disp));
		else if (data.id == "/dev/spidev1.1")
			sensor2_path.lineTo(sensor2_path.currentPosition() + QPointF(data.x_disp, data.y_disp));

		update();
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
		/*sensor1_path = QPainterPath();
		sensor2_path = QPainterPath();
		sensor1_path.moveTo(QPointF(startx, starty));
		sensor2_path.moveTo(QPointF(startx, starty));*/

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

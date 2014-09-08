#include "mouse_monitor_pc/MouseMonitorTrackPathWidget.hpp"

#include <QPainter>
#include <QToolTip>
#include <QMouseEvent>

#include <math.h>

// Direction grid params
#define GRID_SCALE 20

namespace minotaur
{

	void TrackPathWidget::init(double posx, double posy)
	{
		startx = this->width() / 2.0; //posx;
		starty = this->height() / 2.0; //posy;

		sensor1_path.moveTo(QPointF(startx, starty));
		sensor2_path.moveTo(QPointF(startx, starty));

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
		QPainter painter(this);

		drawGrid(painter);

		painter.setRenderHint(QPainter::Antialiasing);
		painter.translate(QPointF(translatex, translatey));
		painter.scale((qreal) zoom, (qreal) zoom);

		drawRobot(painter);

		// Draw path
		if (sensor1_enable) {
			painter.setPen(Qt::blue);
			painter.drawPath(sensor1_path);
		}

		if (sensor2_enable) {
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

		painter.setPen(QPen(Qt::gray, 1));

		for (int i = 0; i <= xSteps; ++i) {
			painter.drawLine(i * GRID_SCALE, 0,
			                 i * GRID_SCALE, ySteps * GRID_SCALE);
		}

		for (int i = 0; i <= ySteps; ++i) {
			painter.drawLine(0, i * GRID_SCALE,
			                 xSteps * GRID_SCALE, i * GRID_SCALE);
		}
	}

	void TrackPathWidget::drawRobot(QPainter &painter)
	{
		// TODO: Add rotation

		double axisHeight = 1;

		painter.setPen(QPen(Qt::black, axisHeight));
		QLineF axis(robot.xPos() - robot.getAttributes().distanceToWheel, robot.yPos(),
		            robot.xPos() + robot.getAttributes().distanceToWheel, robot.yPos());
		painter.drawLine(axis);

		painter.setPen(QPen(Qt::green, axisHeight/2.0));
		QRectF wheel1(robot.xPos() - robot.getAttributes().distanceToWheel - axisHeight/2.0,
		              robot.yPos() - axisHeight, axisHeight, axisHeight*2);
		QRectF wheel2(robot.xPos() + robot.getAttributes().distanceToWheel - axisHeight/2.0,
		              robot.yPos() - axisHeight, axisHeight, axisHeight*2);

		painter.drawRect(wheel1);
		painter.drawRect(wheel2);

		// Draw coordinate system
		/*painter.setPen(QPen(Qt::red, 0.5));
		double len = 4;
		painter.drawLine(robot.xPos() - 0.25, robot.yPos(),
		                  robot.xPos() + (sin(robot.getDirection() * len)),
		                  robot.yPos() + (cos(robot.getDirection() * len)));*/
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
		sensor1_path = QPainterPath();
		sensor2_path = QPainterPath();

		sensor1_path.moveTo(QPointF(startx, starty));
		sensor2_path.moveTo(QPointF(startx, starty));

		translatex = 0.0;
		translatey = 0.0;
		lastMousePos.setX(0.0);
		lastMousePos.setY(0.0);

		update();
	}

	void TrackPathWidget::updateRobot(Robot robot)
	{
		this->robot = robot;
	}

}

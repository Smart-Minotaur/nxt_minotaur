#include "mouse_monitor_pc/MouseMonitorTrackPathWidget.hpp"

#include <QPainter>
#include <QToolTip>
#include <QMouseEvent>

// Direction grid params
#define GRID_SCALE 20

namespace minotaur
{

    void TrackPathWidget::init(double posx, double posy)
    {
        startx = posx;
        starty = posy;

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

        // Draw grid
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

        painter.setRenderHint(QPainter::Antialiasing);
        painter.translate(QPointF(translatex, translatey));
        painter.scale((qreal) zoom, (qreal) zoom);

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

}

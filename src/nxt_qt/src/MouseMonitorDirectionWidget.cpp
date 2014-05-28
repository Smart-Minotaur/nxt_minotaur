#include "nxt_qt/MouseMonitorDirectionWidget.hpp"

#include <QPainter>

// Direction grid params
#define GRID_SCALE 20

namespace minotaur
{
  
    double sensor1_x;
    double sensor1_y;
    double sensor2_x;
    double sensor2_y;

    void DirectionWidget::paintEvent(QPaintEvent *event)
    {
        QPainter painter(this);

        double gridWidth = this->width();
        double gridHeight = this->height();
	double gridCentre = (int) (gridWidth / 2);

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
        
        painter.setPen(QPen(Qt::red, 3));
	painter.drawLine(QPointF(gridCentre, gridCentre), 
			  QPointF(gridCentre + sensor1_x, gridCentre + sensor1_y));
			  
	painter.setPen(QPen(Qt::red, 3));
	painter.drawLine(QPointF(gridCentre, gridCentre), 
			  QPointF(gridCentre + sensor2_x, gridCentre + sensor2_y));
    }

    void DirectionWidget::updateWidget(MouseData data)
    {
        // TODO: Update the data
        if (data.id == SENSOR1)
	{
	  sensor1_x = data.x_disp;
	  sensor1_y = data.y_disp;
	}
	else
	{
	  sensor2_x = data.x_disp;
	  sensor2_y = data.y_disp;
	}
        update();
    }

}

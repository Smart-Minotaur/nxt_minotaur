#include "nxt_qt/MouseMonitorDirectionWidget.hpp"

#include <QPainter>

// Direction grid params
#define GRID_SCALE 20

namespace minotaur
{

    void DirectionWidget::paintEvent(QPaintEvent *event)
    {
        QPainter painter(this);

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

    void DirectionWidget::updateWidget(MouseData data)
    {
        // TODO: Update the data

        update();
    }

}

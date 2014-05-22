#include "nxt_qt/MouseMonitorWindow.hpp"
#include <QPainter>
#include <QWidget>
#include "ros/ros.h"

#define SENSOR1 "/dev/spidev1.0"
#define SENSOR2 "/dev/spidev1.1"

//grid params
#define AMPLIFY 1000
#define GRID_X_START 0
#define GRID_Y_START 0
#define GRID_X_MAX 400
#define GRID_Y_MAX 400
#define GRID_X_MID ((GRID_X_MAX - GRID_X_START) / 2) + GRID_X_START
#define GRID_Y_MID ((GRID_Y_MAX - GRID_Y_START) / 2) + GRID_Y_START
#define SCALE 20

namespace minotaur
{

    MouseMonitorWindow::MouseMonitorWindow(QWidget *parent) :
        QMainWindow(parent)
    {
        setupUi(this);

        widget1 = new DirectionWidget(widget); // widget = name of widget in ui
        widget1->layout()->addWidget(widget1);

        widget2 = new DirectionWidget(widget_2); //widget2 = name of widget in ui
        widget2->layout()->addWidget(widget2);

        connect(&monitorNode, SIGNAL(measuredMouseData(const MouseData)),
                this, SLOT(processMouseData(const MouseData)));
    }

    MouseMonitorWindow::~MouseMonitorWindow()
    {
        delete widget1;
        delete widget2;
    }

    MouseMonitorNode& MouseMonitorWindow::getMonitorNode()
    {
        return monitorNode;
    }

    void MouseMonitorWindow::processMouseData(const MouseData data)
    {
        if (data.id == SENSOR1) {
            widget1->updateWidget(data);
        } else {
            widget2->updateWidget(data);
        }
    }


    void MouseMonitorWindow::paintEvent(QPaintEvent *event)
    {
        /*ROS_INFO("inside mouseMonitorWindow::paintevent");
        QPainter *painter = new QPainter(this);
        painter->begin(this);

        // Add the vertical lines first, paint them
        painter->setPen(QPen(Qt::black, 1));
        for (int x = GRID_X_START; x <= GRID_X_MAX; x += SCALE)
          painter->drawLine(x, GRID_Y_START, x, GRID_Y_MAX);

        // Now add the horizontal lines, paint them
        for (int y = GRID_Y_START; y <= GRID_Y_MAX; y += SCALE)
          painter->drawLine(GRID_X_START, y, GRID_X_MAX, y);
        */
    }


    void DirectionWidget::paintEvent(QPaintEvent *event)
    {
        QPainter *painter = new QPainter(this);
        painter->begin(this);

        // Add the vertical lines first, paint them
        painter->setPen(QPen(Qt::black, 1));
        for (int x = GRID_X_START; x <= GRID_X_MAX; x += SCALE)
            painter->drawLine(x, GRID_Y_START, x, GRID_Y_MAX);

        // Now add the horizontal lines, paint them
        for (int y = GRID_Y_START; y <= GRID_Y_MAX; y += SCALE)
            painter->drawLine(GRID_X_START, y, GRID_X_MAX, y);
    }

    void DirectionWidget::updateWidget(MouseData data)
    {
        this->data = data;
        update();
    }

    /*void MouseMonitorWindow::processMouseSettings(const )
    {

    }*/

}

#ifndef MOUSE_MONITOR_DIRECTION_WIDGET_H
#define MOUSE_MONITOR_DIRECTION_WIDGET_H

#include <QWidget>
#include <QColor>
#include "nxt_qt/MouseMonitorNode.hpp"

#define DEFAULT_AMPLIFY 500

namespace minotaur
{

    class DirectionWidget : public QWidget
    {
            Q_OBJECT

        private:
            double sensor_x;
            double sensor_y;

            double amplify;

            QColor color;

        protected:
            void paintEvent(QPaintEvent *event);

        public:
            DirectionWidget(QColor color, QWidget *parent = 0) :
                QWidget(parent),
                amplify(DEFAULT_AMPLIFY),
                color(color),
                sensor_x(0),
                sensor_y(0) {}

            virtual ~DirectionWidget() {}

            void updateWidget(MouseData data);
    };

}

#endif

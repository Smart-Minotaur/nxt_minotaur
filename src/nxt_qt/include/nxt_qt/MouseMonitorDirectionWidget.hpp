#ifndef MOUSE_MONITOR_DIRECTION_WIDGET_H
#define MOUSE_MONITOR_DIRECTION_WIDGET_H

#include <QWidget>
#include "nxt_qt/MouseMonitorNode.hpp"

namespace minotaur
{

    class DirectionWidget : public QWidget
    {
            Q_OBJECT

        private:
            double sensor1_x;
            double sensor1_y;
            double sensor2_x;
            double sensor2_y;

        protected:
            void paintEvent(QPaintEvent *event);

        public:
            DirectionWidget(QWidget *parent = 0) : QWidget(parent) {}
            virtual ~DirectionWidget() {}

            void updateWidget(MouseData data);
    };

}

#endif

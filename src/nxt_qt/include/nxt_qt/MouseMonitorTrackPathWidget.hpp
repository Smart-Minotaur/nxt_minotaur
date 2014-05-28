#ifndef MOUSE_MONITOR_TRACK_PATH_WIDGET_H
#define MOUSE_MONITOR_TRACK_PATH_WIDGET_H

#include <QWidget>
#include <QPainterPath>

#include "nxt_qt/MouseMonitorNode.hpp"

namespace minotaur
{

    class TrackPathWidget : public QWidget
    {
            Q_OBJECT

        private:
            double startx;
            double starty;

            QPointF lastMousePos;

            double translatex;
            double translatey;

            QPainterPath sensor1_path;
            QPainterPath sensor2_path;

            int zoom;
            bool sensor1_enable;
            bool sensor2_enable;

        protected:
            void paintEvent(QPaintEvent *event);
            void mouseMoveEvent(QMouseEvent *event);
            void mouseReleaseEvent(QMouseEvent *event);

        public:
            TrackPathWidget(QWidget *parent = 0) : QWidget(parent) {}
            virtual ~TrackPathWidget() {}

            void init(double posx, double posy);
            void updateWidget(MouseData data);
            void reset();

        public Q_SLOTS:
            void zoomValueChanged(const int value);
            void sensor1Enable(const int status);
            void sensor2Enable(const int status);
    };

}

#endif

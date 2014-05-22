#ifndef MOUSE_MONITOR_WINDOW_H
#define MOUSE_MONITOR_WINDOW_H

#include <QtGui/QMainWindow>
#include <QWidget>
#include <QPainterPath>
#include <qwt_plot_curve.h>
#include <qwt_plot_layout.h>

#include "nxt_beagle/MouseMonitorConfig.hpp"
#include "ui_mousemonitor_window.h"
#include "nxt_qt/MouseMonitorNode.hpp"

#define SAMPLE_RANGE 300

namespace minotaur
{

    class TrackPathWidget : public QWidget
    {
            Q_OBJECT

        private:
            MouseData data;
            double startx;
            double starty;
            QPainterPath path;

        public:
            TrackPathWidget(QWidget *parent = 0) : QWidget(parent) {
            }

            virtual ~TrackPathWidget() {
            }

            void init() {
                startx = 300;//this->width() / 2;
                starty = 300;//this->height() / 2;

                path.moveTo(QPointF(startx, starty));
            }

            void updateWidget(MouseData data);

        protected:
            void paintEvent(QPaintEvent *event);
    };

    class DirectionWidget : public QWidget
    {
            Q_OBJECT

        private:
            MouseData data;

        public:
            DirectionWidget(QWidget *parent = 0) : QWidget(parent) {}
            virtual ~DirectionWidget() {}

            void updateWidget(MouseData data);

        protected:
            void paintEvent(QPaintEvent *event);
    };

    class MouseMonitorWindow : public QMainWindow, public Ui::MouseMonitorMainWindow
    {
            Q_OBJECT

        private:
            MouseMonitorNode monitorNode;
            DirectionWidget *directionWidget1;
            DirectionWidget *directionWidget2;
            TrackPathWidget *pathWidget;

            // Plot data
            double xSamplesSensor1[SAMPLE_RANGE];
            double xSamplesSensor2[SAMPLE_RANGE];

            double xDisp1[SAMPLE_RANGE];
            double yDisp1[SAMPLE_RANGE];
            double xDisp2[SAMPLE_RANGE];
            double yDisp2[SAMPLE_RANGE];

            QwtPlotCurve xDisp1Curve;
            QwtPlotCurve yDisp1Curve;
            QwtPlotCurve xDisp2Curve;
            QwtPlotCurve yDisp2Curve;

            int sampleCountSensor1;
            int sampleCountSensor2;

            void initWidgets();
            void initTable();
            void initPlots();

            void updatePlotSensor1(MouseData data);
            void updatePlotSensor2(MouseData data);

        private Q_SLOTS:
            void processMouseData(const MouseData data);
            void processMouseSettings(const std::string id,
                                      const pln_minotaur::PLN2033_Settings settings);

        public:
            MouseMonitorWindow(QWidget *parent = 0);
            virtual ~MouseMonitorWindow();

            MouseMonitorNode& getMonitorNode();
    };

}

#endif

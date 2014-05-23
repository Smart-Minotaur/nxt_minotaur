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

        public:
            TrackPathWidget(QWidget *parent = 0) : QWidget(parent) {}
            virtual ~TrackPathWidget() {}

            void init();
            void updateWidget(MouseData data);

        public Q_SLOTS:
            void zoomValueChanged(const int value);
            void sensor1Enable(const int status);
            void sensor2Enable(const int status);

        protected:
            void paintEvent(QPaintEvent *event);
            void mouseMoveEvent(QMouseEvent *event);
            void mouseReleaseEvent(QMouseEvent *event);
    };

    class DirectionWidget : public QWidget
    {
            Q_OBJECT

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

            // Plot stuff
            double xSamplesSensor1[SAMPLE_RANGE];
            double xSamplesSensor2[SAMPLE_RANGE];

            int sampleCountSensor1;
            int sampleCountSensor2;

            double xDisp1[SAMPLE_RANGE];
            double yDisp1[SAMPLE_RANGE];
            double xDisp2[SAMPLE_RANGE];
            double yDisp2[SAMPLE_RANGE];

            double xSpeed1[SAMPLE_RANGE];
            double ySpeed1[SAMPLE_RANGE];
            double xSpeed2[SAMPLE_RANGE];
            double ySpeed2[SAMPLE_RANGE];

            QwtPlotCurve xDisp1Curve;
            QwtPlotCurve yDisp1Curve;
            QwtPlotCurve xDisp2Curve;
            QwtPlotCurve yDisp2Curve;

            QwtPlotCurve xSpeed1Curve;
            QwtPlotCurve ySpeed1Curve;
            QwtPlotCurve xSpeed2Curve;
            QwtPlotCurve ySpeed2Curve;

            void initWidgets();
            void initTable();
            void initPlots();
            void initPlot(QwtPlot *plot,
                          QString title,
                          double maxYRange,
                          double xStep,
                          double yStep,
                          QString xAxisTitle,
                          QString yAxisTitle);

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

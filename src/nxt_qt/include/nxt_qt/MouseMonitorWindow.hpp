#ifndef MOUSE_MONITOR_WINDOW_H
#define MOUSE_MONITOR_WINDOW_H

#include <QtGui/QMainWindow>
#include <QWidget>
#include <QTimer>
#include <QToolButton>

// TODO: Remove that
#include <qwt_plot_curve.h>
#include <qwt_plot_layout.h>

#include "nxt_beagle/MouseMonitorConfig.hpp"
#include "ui_mousemonitor_window.h"
#include "nxt_qt/MouseMonitorNode.hpp"

#include "nxt_qt/MouseMonitorDirectionWidget.hpp"
#include "nxt_qt/MouseMonitorTrackPathWidget.hpp"

#define SAMPLE_RANGE 500

namespace minotaur
{

    class MouseMonitorWindow :
        public QMainWindow,
        public Ui::MouseMonitorMainWindow
    {
            Q_OBJECT

        private:
            MouseMonitorNode monitorNode;

            DirectionWidget *directionWidget1;
            DirectionWidget *directionWidget2;
            TrackPathWidget *pathWidget;

            // Toolbar
            QToolButton *sampleRateBtn;
            QLineEdit *sampleRateEdit;
            QToolButton *resolution1Btn;
            QLineEdit *resolution1Edit;
            QToolButton *resolution2Btn;
            QLineEdit *resolution2Edit;

            QTimer *timer;
            int sampleRateMs;

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

            // Init functions
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
            void initToolbar();
            void initTimer();
            void connectSlots();

            void updatePlotSensor1(MouseData data);
            void updatePlotSensor2(MouseData data);

            QString uintToQString(uint data);

        private Q_SLOTS:
            void processMouseData(const MouseData data);
            void processMouseSettings(const pln_minotaur::PLN2033_Settings settings);

            void openAboutWindow();

            void sampleRateBtnClicked();
            void resolution1BtnClicked();
            void resolution2BtnClicked();

            void detailDebuggingEnable(const int status);

            void getSensorSettingsBtnClicked();

            void timerTimeout();

        public:
            MouseMonitorWindow(QWidget *parent = 0);
            virtual ~MouseMonitorWindow();

            MouseMonitorNode& getMonitorNode();
    };

}

#endif

#ifndef MOUSE_MONITOR_WINDOW_H
#define MOUSE_MONITOR_WINDOW_H

#include <QtGui/QMainWindow>
#include <QWidget>
#include <QTimer>
#include <QToolButton>

#include "nxt_beagle/MouseMonitorConfig.hpp"
#include "ui_mousemonitor_window.h"
#include "nxt_qt/MouseMonitorNode.hpp"

#include "nxt_qt/MouseMonitorDirectionWidget.hpp"
#include "nxt_qt/MouseMonitorTrackPathWidget.hpp"
#include "nxt_qt/MouseMonitorPlot.hpp"

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
            MouseMonitorPlot *xDisp1Plot;
            MouseMonitorPlot *yDisp1Plot;
            MouseMonitorPlot *xDisp2Plot;
            MouseMonitorPlot *yDisp2Plot;

            MouseMonitorPlot *xSpeed1Plot;
            MouseMonitorPlot *ySpeed1Plot;
            MouseMonitorPlot *xSpeed2Plot;
            MouseMonitorPlot *ySpeed2Plot;

            // TODO
            MouseMonitorPlot *detail1Plot;
            MouseMonitorPlot *detail2Plot;

            // Init functions
            void initWidgets();
            void initTable();
            void initPlots();
            void initToolbar();
            void initTimer();
            void initDetail();
            void connectSlots();

            void updateAbsValue(MouseData data);
            void updatePlot(MouseData data);
            void updateDirectionWidgets(MouseData data);
            void updateData(MouseData data);

            QString uintToQString(uint data);

            void processMouseData(const MouseData data);
            void processMouseSettings(const pln_minotaur::PLN2033_Settings settings);

        private Q_SLOTS:
            void openAboutWindow();

            void sampleRateBtnClicked();
            void resolution1BtnClicked();
            void resolution2BtnClicked();
            void getSensorSettingsBtnClicked();
            void trackPathResetBtnClicked();
            void getData1BtnClicked();
            void getData2BtnClicked();

            void detailDebuggingEnable(const int status);

            void timerTimeout();

        public:
            MouseMonitorWindow(QWidget *parent = 0);
            virtual ~MouseMonitorWindow();

            MouseMonitorNode& getMonitorNode();
      
    };

}

#endif

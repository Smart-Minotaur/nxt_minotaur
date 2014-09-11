#ifndef MOUSE_MONITOR_WINDOW_H
#define MOUSE_MONITOR_WINDOW_H

#include <QtGui/QMainWindow>
#include <QWidget>
#include <QTimer>
#include <QToolButton>

#include "ui_mousemonitor_window.h"

#include "mouse_monitor_beagle/MouseMonitorConfig.hpp"

#include "mouse_monitor_pc/MouseMonitorNode.hpp"
#include "mouse_monitor_pc/MouseMonitorDirectionWidget.hpp"
#include "mouse_monitor_pc/MouseMonitorTrackPathWidget.hpp"
#include "mouse_monitor_pc/MouseMonitorPlot.hpp"
#include "mouse_monitor_pc/MouseMonitorMedianFilterDialog.hpp"
#include "mouse_monitor_pc/MouseMonitorLogDialog.hpp"
#include "mouse_monitor_pc/MouseMonitorCalibrationWizard.hpp"
#include "mouse_monitor_pc/MouseMonitorCalibrationData.hpp"

#include "minotaur_common/MedianFilter.hpp"

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

			MouseMonitorMedianFilterDialog *medianFilterDialog;
			MedianFilter *medianFilter_sensor1_yDisp;
			MedianFilter *medianFilter_sensor1_xDisp;
			MedianFilter *medianFilter_sensor2_yDisp;
			MedianFilter *medianFilter_sensor2_xDisp;

			MouseMonitorLogDialog *logDialog;
			MouseMonitorCalibrationWizard *calibrationWizard;
			MouseMonitorCalibrationData calibrationData;

			DirectionWidget *directionWidget1;
			DirectionWidget *directionWidget2;
			TrackPathWidget *pathWidget;

			QTimer *timer;
			int sampleRate;
			
			Robot robot;

			// Toolbar
			QToolButton *sampleRateBtn;
			QLineEdit *sampleRateEdit;
			QToolButton *resolution1Btn;
			QLineEdit *resolution1Edit;
			QToolButton *resolution2Btn;
			QLineEdit *resolution2Edit;

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
			void initMedianFilter();
			void initSensorCalibration();
			void connectSlots();

			/**
			 * This function is called to process the received data and
			 * update all display widgets with the new sensor data.
			 */
			void processMouseData(const MouseData data);

			/**
			* This function is used to update the table with the sensors settings.
			*/
			void processMouseSettings(const pln_minotaur::PLN2033_Settings settings);

			MouseData correctMouseData(MouseData data);
			void applyMedianFilter(MouseData &data);
			void calibrate(MouseData &data);

			void updateDataDisplay(MouseData data);
			void updateAbsoluteValueDisplay(MouseData data);
			void updatePlots(MouseData data);
			void updateDirectionWidgets(MouseData data);

			QString uintToQString(uint data);
			int sampleRateToInterval(int sampleRate);

		private Q_SLOTS:
			void openCalibrateSensorsWizard();
			void openAboutWindow();
			void openLogDialog();
			void openMedianFilterSettingsDialog();

			void sampleRateBtnClicked();
			void resolution1BtnClicked();
			void resolution2BtnClicked();
			void getSensorSettingsBtnClicked();
			void trackPathResetBtnClicked();
			void getData1BtnClicked();
			void getData2BtnClicked();
			void resetGraphsDispBtnClicked();
			void resetGraphsSpeedBtnClicked();

			// Median filter settings dialog
			void newMedianFilterSettings(MedianFilterSettings settings);
			void clearMedianFilterClicked();
			
			// Calibration wizard
			void startCalibrateSensors(MouseMonitorCalibrationData data);
			void stopCalibrateSensors();

			void detailDebuggingEnable(const int status);

			void timerTimeout();

		public:
			MouseMonitorWindow(QWidget *parent = 0);
			virtual ~MouseMonitorWindow();

			MouseMonitorNode& getMonitorNode();
	};

}

#endif

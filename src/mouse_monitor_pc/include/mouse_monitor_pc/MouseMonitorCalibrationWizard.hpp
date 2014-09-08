#ifndef MOUSEMONITORCALIBRATIONWIZARD_HPP
#define MOUSEMONITORCALIBRATIONWIZARD_HPP

#include <QWizard>

#include "ui_mousemonitor_sensorCalibration.h"
#include "mouse_monitor_pc/MouseMonitorCalibrationData.hpp"

namespace minotaur
{

	class MouseMonitorCalibrationWizard : public QWizard, public Ui::sensorCalibration
	{
			Q_OBJECT

		private:
			MouseMonitorCalibrationData calibrationData;
			bool calibrationFinished;

		public:
			MouseMonitorCalibrationWizard(QWidget * parent = 0);
			~MouseMonitorCalibrationWizard() { };

			void setMouseMonitorCalibrationData(MouseMonitorCalibrationData calibrationData);
			MouseMonitorCalibrationData getMouseMonitorCalibrationData() const;

		private Q_SLOTS:
			void startBtnClicked();
			void stopBtnClicked();
			virtual bool validateCurrentPage();
			
		Q_SIGNALS:
			void startCalibrateSensors(MouseMonitorCalibrationData data);
			void stopCalibrateSensors();
	};

}

#endif // MOUSEMONITORCALIBRATIONWIZARD_HPP

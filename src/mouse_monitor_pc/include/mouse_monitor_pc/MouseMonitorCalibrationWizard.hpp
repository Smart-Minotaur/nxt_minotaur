#ifndef MOUSEMONITORCALIBRATIONWIZARD_HPP
#define MOUSEMONITORCALIBRATIONWIZARD_HPP

#include <QWizard>

#include "ui_mousemonitor_sensorCalibration.h"

namespace minotaur
{

class MouseMonitorCalibrationWizard : public QWizard, public Ui::sensorCalibration
{
	Q_OBJECT
	
public:
	MouseMonitorCalibrationWizard(QWidget * parent = 0);
	~MouseMonitorCalibrationWizard() { };
};

}

#endif // MOUSEMONITORCALIBRATIONWIZARD_HPP

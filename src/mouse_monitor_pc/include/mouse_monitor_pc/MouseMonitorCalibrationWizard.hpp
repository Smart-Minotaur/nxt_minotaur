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
/*
class MouseMonitorCalibrationWizardDistancePage : public QWizardPage, public Ui::distancePage1
{
	Q_OBJECT
	
public: 
	MouseMonitorCalibrationWizardDistancePage(QWidget * parent = 0);
	~MouseMonitorCalibrationWizardDistancePage() { };
private:

};

class MouseMonitorCalibrationWizardMeasurementPage : public QWizardPage, public Ui::measurePage2
{
	Q_OBJECT
	
	public:
	MouseMonitorCalibrationWizardMeasurementPage(QWidget * parent = 0);
	~MouseMonitorCalibrationWizardMeasurementPage() { };
	
	private:
	float s1XEditValue;
};

class MouseMonitorCalibrationWizardResultPage : public QWizardPage, public Ui::resultPage3
{
	Q_OBJECT
	
	public:
	MouseMonitorCalibrationWizardResultPage(QWidget * parent = 0);
	~MouseMonitorCalibrationWizardResultPage() { };
	
	private:

};
*/


}

#endif // MOUSEMONITORCALIBRATIONWIZARD_HPP

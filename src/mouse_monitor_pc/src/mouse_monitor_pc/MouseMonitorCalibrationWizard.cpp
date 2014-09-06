#include "mouse_monitor_pc/MouseMonitorCalibrationWizard.hpp"

namespace minotaur
{

MouseMonitorCalibrationWizard::MouseMonitorCalibrationWizard(QWidget *parent)
     : QWizard(parent)
{
	setupUi(this);	
	/*addPage(new MouseMonitorCalibrationWizardDistancePage);
    addPage(new MouseMonitorCalibrationWizardMeasurementPage);
    addPage(new MouseMonitorCalibrationWizardResultPage);
	*/
}

/*
MouseMonitorCalibrationWizardDistancePage::MouseMonitorCalibrationWizardDistancePage(QWidget *parent) 
	: QWizardPage(parent)
{
}



MouseMonitorCalibrationWizardMeasurementPage::MouseMonitorCalibrationWizardMeasurementPage(QWidget *parent)
	: QWizardPage(parent)
{
}

MouseMonitorCalibrationWizardResultPage::MouseMonitorCalibrationWizardResultPage(QWidget *parent)
	: QWizardPage(parent)
{
}

*/
}


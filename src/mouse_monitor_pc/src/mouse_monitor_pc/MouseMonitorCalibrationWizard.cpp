#include "mouse_monitor_pc/MouseMonitorCalibrationWizard.hpp"

namespace minotaur
{

	MouseMonitorCalibrationWizard::MouseMonitorCalibrationWizard(QWidget *parent)
		: QWizard(parent)
	{
		setupUi(this);
		calibrationFinished = false;
		connect(startBtn, SIGNAL(clicked()), this, SLOT(startBtnClicked()));
		connect(stopBtn, SIGNAL(clicked()), this, SLOT(stopBtnClicked()));
		//connect(sailing, SIGNAL(selectionChanged()), this, SIGNAL(completeChanged()));
	}

	void MouseMonitorCalibrationWizard::setMouseMonitorCalibrationData(MouseMonitorCalibrationData calibrationData)
	{
		this->calibrationData = calibrationData;

		s1XEdit->setText(QString::number(this->calibrationData.getS1XDisplacement()));
		s1YEdit->setText(QString::number(this->calibrationData.getS1YDisplacement()));
		s2XEdit->setText(QString::number(this->calibrationData.getS2XDisplacement()));
		s2YEdit->setText(QString::number(this->calibrationData.getS2YDisplacement()));

		s1AngleOffsetEdit->setText(QString::number(this->calibrationData.getS1AngleOffset()));
		s2AngleOffsetEdit->setText(QString::number(this->calibrationData.getS2AngleOffset()));

		s1RealDistanceEdit->setText(QString::number(this->calibrationData.getS1RealDistance()));
		s2RealDistanceEdit->setText(QString::number(this->calibrationData.getS2RealDistance()));
	}
	MouseMonitorCalibrationData MouseMonitorCalibrationWizard::getMouseMonitorCalibrationData() const
	{
		return calibrationData;
	}

	void MouseMonitorCalibrationWizard::startBtnClicked()
	{
		calibrationFinished = false;
		Q_EMIT startCalibrateSensors(this->calibrationData);
	}

	void MouseMonitorCalibrationWizard::stopBtnClicked()
	{
		calibrationFinished = true;
		Q_EMIT stopCalibrateSensors();
	}

	bool MouseMonitorCalibrationWizard::validateCurrentPage()
	{
		if(calibrationFinished)
		{
			return true;
		}
		return false;
	}

}

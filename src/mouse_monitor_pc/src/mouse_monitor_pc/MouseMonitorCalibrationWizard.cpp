#include "mouse_monitor_pc/MouseMonitorCalibrationWizard.hpp"

namespace minotaur
{

	MouseMonitorCalibrationWizard::MouseMonitorCalibrationWizard(QWidget *parent)
		: QWizard(parent)
	{
		setupUi(this);

		connect(startBtn, SIGNAL(clicked()), this, SLOT(startBtnClicked()));
		connect(stopBtn, SIGNAL(clicked()), this, SLOT(stopBtnClicked()));
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
		Q_EMIT startCalibrateSensors(this->calibrationData);
	}

	void MouseMonitorCalibrationWizard::stopBtnClicked()
	{
		Q_EMIT stopCalibrateSensors();
	}

}

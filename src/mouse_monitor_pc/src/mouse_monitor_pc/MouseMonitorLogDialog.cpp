#include "mouse_monitor_pc/MouseMonitorLogDialog.hpp"

namespace minotaur
{

	MouseMonitorLogDialog::MouseMonitorLogDialog(QWidget *parent) :
		QDialog(parent)
	{
		setupUi(this);

		connect(clearBtn, SIGNAL(clicked()), this, SLOT(clearBtnClicked()));
	}

	MouseMonitorLogDialog::~MouseMonitorLogDialog()
	{
	}

	void MouseMonitorLogDialog::clearBtnClicked()
	{
		s1YEdit_raw->clear();
		s1XEdit_raw->clear();
		s2YEdit_raw->clear();
		s2XEdit_raw->clear();
		s1YEdit_filtered->clear();
		s1XEdit_filtered->clear();
		s2YEdit_filtered->clear();
		s2XEdit_filtered->clear();
	}

	void MouseMonitorLogDialog::logRaw(MouseData data)
	{
		if (data.id == SENSOR1) {
			if (data.y_disp != 0)
				s1YEdit_raw->appendPlainText(QString::number(data.y_disp));
			if (data.x_disp != 0)
				s1XEdit_raw->appendPlainText(QString::number(data.x_disp));
		} else if (data.id == SENSOR2) {
			if (data.y_disp != 0)
				s2YEdit_raw->appendPlainText(QString::number(data.y_disp));
			if (data.x_disp != 0)
				s2XEdit_raw->appendPlainText(QString::number(data.x_disp));
		}
	}

	void MouseMonitorLogDialog::logFiltered(MouseData data)
	{
		if (data.id == SENSOR1) {
			if (data.y_disp != 0)
				s1YEdit_filtered->appendPlainText(QString::number(data.y_disp));
			if (data.x_disp != 0)
				s1XEdit_filtered->appendPlainText(QString::number(data.x_disp));
		} else if (data.id == SENSOR2) {
			if (data.y_disp != 0)
				s2YEdit_filtered->appendPlainText(QString::number(data.y_disp));
			if (data.x_disp != 0)
				s2XEdit_filtered->appendPlainText(QString::number(data.x_disp));
		}
	}

}

#include "nxt_qt/MouseMonitorWindow.hpp"

namespace minotaur
{

	MouseMonitorWindow::MouseMonitorWindow(QWidget *parent) :
		QMainWindow(parent)
	{
		setupUi(this);

		connect(&monitorNode, SIGNAL(measuredMouseData(const MouseData)),
			this, SLOT(processMouseData(const MouseData)));
	}

	MouseMonitorWindow::~MouseMonitorWindow() {}

	MouseMonitorNode& MouseMonitorWindow::getMonitorNode()
	{
		return monitorNode;
	}

	void MouseMonitorWindow::processMouseData(const MouseData data)
	{
		x_disp1->setText(QString::fromStdString(data.id));		
	}

	/*void MouseMonitorWindow::processMouseSettings(const )
	{

	}*/

}

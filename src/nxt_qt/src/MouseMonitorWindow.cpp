#include "nxt_qt/MouseMonitorWindow.hpp"

namespace minotaur
{

	MouseMonitorWindow::MouseMonitorWindow(QWidget *parent)
	: QMainWindow(parent)
	{
		setupUi(this);
	}

	MouseMonitorWindow::~MouseMonitorWindow() {}

}

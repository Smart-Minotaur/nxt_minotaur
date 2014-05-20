#ifndef MOUSE_MONITOR_WINDOW_H
#define MOUSE_MONITOR_WINDOW_H

#include <QtGui/QMainWindow>
#include "ui_mousemonitor_window.h"

namespace minotaur
{

	class MouseMonitorWindow : public QMainWindow, public Ui::MouseMonitorMainWindow
	{
		Q_OBJECT

	//	private:
	//	private Q_SLOTS:

		public:
			MouseMonitorWindow(QWidget *parent = 0);
			~MouseMonitorWindow();

		// QMouseNode getMouseNode();
	};

}

#endif

#ifndef MOUSE_MONITOR_WINDOW_H
#define MOUSE_MONITOR_WINDOW_H

#include <QtGui/QMainWindow>
#include "ui_mousemonitor_window.h"
#include "nxt_qt/MouseMonitorNode.hpp"

namespace minotaur
{

	class MouseMonitorWindow : public QMainWindow, public Ui::MouseMonitorMainWindow
	{
		Q_OBJECT

		private:
			MouseMonitorNode monitorNode;

		private Q_SLOTS:
			void processMouseData(const MouseData data);

		public:
			MouseMonitorWindow(QWidget *parent = 0);
			virtual ~MouseMonitorWindow();
		
		MouseMonitorNode& getMonitorNode();
	};

}

#endif

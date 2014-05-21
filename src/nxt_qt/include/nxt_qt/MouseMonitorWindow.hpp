#ifndef MOUSE_MONITOR_WINDOW_H
#define MOUSE_MONITOR_WINDOW_H

#include <QtGui/QMainWindow>
#include "ui_mousemonitor_window.h"
#include "nxt_qt/MouseMonitorNode.hpp"
#include <QWidget>

namespace minotaur
{
	class DirectionWidget : public QWidget
	{
	  private:
	  MouseData data;
	    
	  public:
	    MyWidget(QWidget *parent = 0):QWidget(parent) {}
	    ~MyWidget() { }
	    
	    void updateWidget(MouseData data);
	    
	    void paintEvent(QPaintEvent *event);
	};
	
	class MouseMonitorWindow : public QMainWindow, public Ui::MouseMonitorMainWindow
	{
		Q_OBJECT

		private:
			MouseMonitorNode monitorNode;
			MyWidget *widget1;
			MyWidget2 *widget2;

		private Q_SLOTS:
			void processMouseData(const MouseData data);

		public:
			MouseMonitorWindow(QWidget *parent = 0);
			virtual ~MouseMonitorWindow();
		
			MouseMonitorNode& getMonitorNode();
			void paintEvent(QPaintEvent *event);
	};
	
	
	

}

#endif

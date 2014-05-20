#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "nxt_qt/QMouseNode.h"
#include "nxt_beagle/DebugMouseData.h"

#include <QMainWindow>
#include <QWidget>
#include <QObject>
#include <QThread>
#include <QMetaType>
#include <QtGui/QMainWindow>
#include <QApplication>

namespace minotaur {

	class MainWindow : public QMainWindow
	{
		Q_OBJECT

		private:
			QMouseNode mouseNode;

		public:
			MainWindow(QWidget *parent = 0);
			~MainWindow();

			QMouseNode& getMouseNode();
			void paintEvent(QPaintEvent *event);
			std::string convertInt(double number);

		public Q_SLOTS:
			void updateMouseMovement(
				double x_speed1, double y_speed1, 
				double x_disp1, double y_disp1,
				double x_speed2, double y_speed2, 
				double x_disp2, double y_disp2);
	};

}

#endif // MAINWINDOW_H

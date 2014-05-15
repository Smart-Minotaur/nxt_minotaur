#ifndef MAINWINDOW_H
#define MAINWINDOW_H

//#include "ui_mainwindow_debug.h"
#include <QMainWindow>
#include "QWidget"
#include "nxt_beagle/DebugMouseData.h"
#include "ros/ros.h"
#include <QObject>
#include "nxt_qt/QMouseNode.h"
#include <QThread>
#include <QMetaType>
#include <QtGui/QMainWindow>
#include <QApplication>

namespace Ui {
class MainWindow;
}

//namespace minotaur {

class MainWindow : public QMainWindow//, public Ui::MainWindow
{
    Q_OBJECT

private:
    Ui::MainWindow *ui;
    QMouseNode mouseNode;
    

public:
    /*explicit*/ MainWindow(QWidget *parent = 0);
    ~MainWindow();
	
    QMouseNode& getMouseNode();
    void paintEvent(QPaintEvent *event);
    //void chatterCallbackGUI(const nxt_qt::DebugMouse& mouse);
    std::string convertInt(double number);
	

public Q_SLOTS:
    void updateMouseMovement(double x_speed1, double y_speed1, 
			     double x_disp1, double y_disp1,
			     double x_speed2, double y_speed2, 
			     double x_disp2, double y_disp2);
    //void setInitInterval();

};

//}


#endif // MAINWINDOW_H

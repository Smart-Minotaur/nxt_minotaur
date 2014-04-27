#include "nxt_qt/mainwindow.h"
#include "ui_mainwindow_debug.h"
#include "QWidget"
#include <QApplication>
#include <QtGui>
#include <QPainter>
#include <QtGui/QMainWindow>
#include <QMetaType>
#include <QLine>
#include "ros/ros.h"
#include <QThread>
#include "nxt_qt/QMouseNode.h"
#include <QPixmap>
#include <QGraphicsScene>
#include "std_msgs/String.h"
#include <string>
#include <QString>


//using namespace Qt;

//namespace minotaur {

//class MainWindow;
int globalx;
int globaly;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

	connect(&mouseNode, SIGNAL(sendMouseMovement(int, int)), this, 
		SLOT(updateMouseMovement(int, int)));
	
	connect(ui->lineEdit_mouse_x, SIGNAL(valueChanged(int)), this,
		SLOT(updateMouseMovement(int, int)));
	connect(ui->lineEdit_mouse_x, SIGNAL(valueChanged(int)), this,
		SLOT(updateMouseMovement(int, int)));
    
}


MainWindow::~MainWindow()
{
    delete ui;
}


QMouseNode& MainWindow::getMouseNode()
{
	return mouseNode;
}


void MainWindow::paintEvent(QPaintEvent *event)
{
    ROS_INFO("Paintevent");
    QPainter *painter = new QPainter (this);
    painter->begin(this);
    
    // Add the vertical lines first, paint them
    painter->setPen(QPen(Qt::black, 1));
    for (int x=100; x<=500; x+=20)
      painter->drawLine(x,100,x,500);

    // Now add the horizontal lines, paint them
    for (int y=100; y<=500; y+=20)
      painter->drawLine(100,y,500,y);
    
    painter->setPen(QPen(Qt::red, 3));
    painter->drawLine (QPointF(300, 300), QPointF(300+globalx, 300-globaly));
    painter->end();
}

void MainWindow::updateMouseMovement(int x, int y)
{
  double speed = sqrt((x*x) + (y*y));
  
  std::string result_x = convertInt(x);
  std::string result_y = convertInt(y);
  QString qstr_x = QString::fromStdString(result_x);
  QString qstr_y = QString::fromStdString(result_y);
  std::string result_speed = convertInt(speed);
  QString qstr_speed = QString::fromStdString(result_speed);
  
  
  ROS_INFO("updatemousemovement [X:%d, Y:%d]", x, y);
  ROS_INFO("Speed: %f", speed);
  ui->lineEdit_mouse_x->setText(qstr_x);
  ui->lineEdit_mouse_y->setText(qstr_y);
  ui->lineEdit_speed->setText(qstr_speed);
  globalx = x;
  globaly = y;
  repaint();
}

std::string MainWindow::convertInt(int number)
{
   std::ostringstream ss;//create a stringstream
   ss << number;//add number to the stream
   return ss.str();//return a string with the contents of the stream
}

//}





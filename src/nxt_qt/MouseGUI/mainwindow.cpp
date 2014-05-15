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
//#include "nxt_qt/QMouseNode.h"
#include <QPixmap>
#include <QGraphicsScene>
#include "std_msgs/String.h"
#include <string>
#include <QString>


//using namespace Qt;

//namespace minotaur {

//class MainWindow;
double globalx1;
double globaly1;
double globalx2;
double globaly2;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

	connect(&mouseNode, SIGNAL(sendMouseMovement(double, double, double, double
	  double, double, double, double)), this, 
	 SLOT(updateMouseMovement(double, double, double, double, 
				  double, double, double, double)));
	
	//connect(ui->lineEdit_mouse_x, SIGNAL(valueChanged(double)), this,
	//	SLOT(updateMouseMovement(double, double)));
	//connect(ui->lineEdit_mouse_x, SIGNAL(valueChanged(double)), this,
	//	SLOT(updateMouseMovement(double, double)));
    
}


MainWindow::~MainWindow()
{
    delete ui;
}


QMouseNode& MainWindow::getMouseNode()
{
	return mouseNode;
}

int x_start = 50;
int y_start = 100;
int x_max = 450;
int y_max = 500;
int x_mitte = ((x_max - x_start) / 2) + x_start;
int y_mitte = ((y_max - y_start) / 2) + y_start;
int scale = 20;
int offset_second_grid = 600;


void MainWindow::paintEvent(QPaintEvent *event)
{
    int amp = 100;
    
    QPainter *painter = new QPainter (this);
    painter->begin(this);
    
    // Add the vertical lines first, paint them
    painter->setPen(QPen(Qt::black, 1));
    for (int x=x_start; x<=x_max; x+=scale)
      painter->drawLine(x,y_start,x,y_max);

    // Now add the horizontal lines, paint them
    for (int y=y_start; y<=y_max; y+=scale)
      painter->drawLine(x_start,y,x_max,y);
    
    //draw second grid
    // Add the vertical lines first, paint them
    for (int x=x_start + offset_second_grid; x<=x_max + offset_second_grid; x+=scale)
      painter->drawLine(x,y_start,x,y_max);

    // Now add the horizontal lines, paint them
    for (int y=y_start; y<=y_max; y+=scale)
      painter->drawLine(x_start + offset_second_grid,y,x_max + offset_second_grid,y);
    
    painter->setPen(QPen(Qt::red, 3));
    painter->drawLine (QPointF(x_mitte, y_mitte), 
		       QPointF(x_mitte + globalx1*amp, y_mitte - globaly2*amp));
    
    painter->setPen(QPen(Qt::red, 3));
    painter->drawLine (QPointF(x_mitte+offset_second_grid, y_mitte), 
		       QPointF(x_mitte + offset_second_grid + globalx1*amp, y_mitte - globaly2*amp));
    painter->end();
}

void MainWindow::updateMouseMovement(double x_speed1, double y_speed1, 
			     double x_disp1, double y_disp1,
			     double x_speed2, double y_speed2, 
			     double x_disp2, double y_disp2)
{
  
  std::string result_x = convertInt(x_disp1);
  std::string result_y = convertInt(y_disp1);
  QString qstr_x = QString::fromStdString(result_x);
  QString qstr_y = QString::fromStdString(result_y);
  std::string result_speed1 = convertInt(x_speed1);
  QString qstr_speed1 = QString::fromStdString(result_speed1);
  std::string result_speed2 = convertInt(x_speed2);
  QString qstr_speed2 = QString::fromStdString(result_speed2);
  
  //ui->lineEdit_mouse_x->setText(qstr_x);
  ui->lineEdit_mouse_x->setText(QString("%1").arg(x_speed1,0,'f',2));
  ui->lineEdit_mouse_y->setText(qstr_y);
  ui->lineEdit_speed_x->setText(qstr_speed1);
  ui->lineEdit_speed_y->setText(qstr_speed2);
  globalx1 = x_disp1;
  globaly1 = y_disp1;
  
  ui->lineEdit_mouse_x_2->setText(QString("%1").arg(x_disp2,0,'f',2));
  ui->lineEdit_mouse_y_2->setText(QString("%1").arg(y_disp2,0,'f',2));
  ui->lineEdit_speed_x_2->setText(QString("%1").arg(x_speed2,0,'f',2));
  ui->lineEdit_speed_y_2->setText(QString("%1").arg(y_speed2,0,'f',2));
  globalx2 = x_disp2;
  globaly2 = y_disp2;
  
  //repaint();
  update();
}

std::string MainWindow::convertInt(double number)
{
   std::ostringstream ss;//create a stringstream
   ss << number;//add number to the stream
   return ss.str();//return a string with the contents of the stream
}

//}





#include "nxt_qt/MapWindow.hpp"
#include <QLayout>

#define MAX_LIN_VEL 0.25f
#define MAX_ANG_VEL 2.0f

namespace minotaur
{
    MapWindow::MapWindow(QWidget *parent)
    :QMainWindow(parent)
    {
        setupUi(this);
        
        linVel = 0;
        angVel = 0,
        
        mapWidget = new QMapWidget(centralwidget);
        centralwidget->layout()->addWidget(mapWidget);
        
        connect(&navNode, SIGNAL(odometryUpdated(const QOdometry)), this, SLOT(processOdometry(const QOdometry)));
        connect(&navNode, SIGNAL(measuredSensor(const QUltraSensor)), this, SLOT(processMeasuredSensor(const QUltraSensor)));
    }
    
    MapWindow::~MapWindow()
    {
        delete mapWidget;
    }
    
    void MapWindow::processOdometry(const QOdometry p_odom)
    {
        robotPosition.point.x = p_odom.x;
        robotPosition.point.y = p_odom.y;
        robotPosition.theta = p_odom.theta;
        
        mapWidget->setPosition(robotPosition);
    }
    
    void MapWindow::processMeasuredSensor(const QUltraSensor p_sensor)
    {
        int id;
        if(p_sensor.direction == 0)
            id = MAP_CREATOR_FRONT_SENSOR;
        else if(p_sensor.direction > 0)
            id = MAP_CREATOR_LEFT_SENSOR;
        else
            id = MAP_CREATOR_RIGHT_SENSOR;
        
        mapWidget->step(id, p_sensor.distance);
        
        std::stringstream ss;
        ss << "(" << std::setprecision(2) << robotPosition.point.x << "/" << std::setprecision(2) << robotPosition.point.y << ")";
        
        RobotPositionLabel->setText(ss.str().c_str());
        mapWidget->update();
    }
    
    QMinotaurNavigateNode& MapWindow::getNavigationNode()
    {
        return navNode;
    }
    
    void MapWindow::keyPressEvent(QKeyEvent *p_event)
    {
        if(p_event->key() == Qt::Key_Up)
           linVel += MAX_LIN_VEL;
        
        if(p_event->key() == Qt::Key_Down)
            linVel -= MAX_LIN_VEL;
        
        if(p_event->key() == Qt::Key_Left)
            angVel += MAX_ANG_VEL;

        if(p_event->key() == Qt::Key_Right)
            angVel -= MAX_ANG_VEL;
        
        navNode.setRobotVelocity(linVel, angVel);
    }
    
    void MapWindow::keyReleaseEvent(QKeyEvent *p_event)
    {
        if(p_event->key() == Qt::Key_Up)
           linVel -= MAX_LIN_VEL;
        
        if(p_event->key() == Qt::Key_Down)
            linVel += MAX_LIN_VEL;
        
        if(p_event->key() == Qt::Key_Left)
            angVel -= MAX_ANG_VEL;

        if(p_event->key() == Qt::Key_Right)
            angVel += MAX_ANG_VEL;
        
        navNode.setRobotVelocity(linVel, angVel);
    }
}
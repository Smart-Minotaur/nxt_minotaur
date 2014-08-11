#include <QLayout>
#include "map_monitor/MapWindow.hpp"
#include "minotaur_common/Math.hpp"

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
        
        sensorSettings = loadCurrentSensorSettings();
        
        connect(&controlNode, SIGNAL(receivedOdometry(const QOdometry)), this, SLOT(processOdometry(const QOdometry)));
        connect(&controlNode, SIGNAL(receivedUltrasonicData(const QUltrasonicData)), this, SLOT(processMeasuredSensor(const QUltrasonicData)));
    }
    
    MapWindow::~MapWindow()
    {
        delete mapWidget;
    }
    
    void MapWindow::processOdometry(const QOdometry p_odom)
    {
        robotPosition.point.x = p_odom.odometry.pose.pose.position.x;
        robotPosition.point.y = p_odom.odometry.pose.pose.position.y;
        robotPosition.theta = getTheta(p_odom.odometry);
        
        mapWidget->setPosition(robotPosition);
    }
    
    void MapWindow::processMeasuredSensor(const QUltrasonicData p_sensor)
    {
        int id;
        if(sensorSettings[p_sensor.data.sensorID].direction == 0)
            id = MAP_CREATOR_FRONT_SENSOR;
        else if(sensorSettings[p_sensor.data.sensorID].direction > 0 && sensorSettings[p_sensor.data.sensorID].direction < M_PI)
            id = MAP_CREATOR_LEFT_SENSOR;
        else
            id = MAP_CREATOR_RIGHT_SENSOR;
        
        mapWidget->step(id, p_sensor.data.distance);
        
        std::stringstream ss;
        ss << "(" << std::setprecision(2) << robotPosition.point.x << "/" << std::setprecision(2) << robotPosition.point.y << ")";
        
        RobotPositionLabel->setText(ss.str().c_str());
        mapWidget->update();
    }
    
    QMinotaurControlNode& MapWindow::getControlNode()
    {
        return controlNode;
    }
}

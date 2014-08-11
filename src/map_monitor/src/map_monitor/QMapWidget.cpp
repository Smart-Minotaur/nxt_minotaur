#include <QSize>
#include <QMessageBox>
#include <QString>
#include <stdexcept>
#include <sstream>
#include "map_monitor/QMapWidget.hpp"
#include "minotaur_common/RAIILock.hpp"

#define MAX_FIELD_VALUE 50
#define MIN_FIELD_VALUE 0

namespace minotaur
{
    QMapWidget::QMapWidget(QWidget *parent)
    : QWidget(parent)
    {
        pthread_mutex_init(&mapMutex, NULL);
        pthread_mutex_init(&positionMutex, NULL);
        
        positionCounter = 0;
        setBackgroundRole(QPalette::Base);
        setAutoFillBackground(true);
        
        setMinimumSize(QSize(mapCreator.getMap()->getWidth(), mapCreator.getMap()->getHeight()));
    }
    
    QMapWidget::~QMapWidget()
    {
        pthread_mutex_destroy(&positionMutex);
        pthread_mutex_destroy(&mapMutex);
    }
    
    void QMapWidget::paintEvent(QPaintEvent *event)
    {
        QPainter painter(this);
        
        painter.setBrush(QBrush(Qt::white));
        painter.drawRect(0, 0, width() - 1, height() - 1);
        
        paintMap(painter);
        paintRobot(painter);
    }
    
    void QMapWidget::paintMap(QPainter& p_painter)
    {
        RAIILock lock(&mapMutex);
        
        QPoint delta;
        delta.setX((width() - mapCreator.getMap()->getWidth()) / 2);
        delta.setY((height() - mapCreator.getMap()->getHeight()) / 2);
        
        p_painter.setPen(Qt::red);
        p_painter.setBrush(QBrush(Qt::red));
        for(int x = 0; x < mapCreator.getMap()->getWidth(); ++x)
        {
            for(int y = 0; y < mapCreator.getMap()->getHeight(); ++y)
            {
                float val = mapCreator.getMap()->getField()[x][y];
                if(val > MAX_FIELD_VALUE)
                    val = MAX_FIELD_VALUE;
                val = (val - MIN_FIELD_VALUE) / (MAX_FIELD_VALUE - MIN_FIELD_VALUE);
                if(val > 0)
                {
                    p_painter.setOpacity(val);
                    p_painter.drawRect(delta.x() + x, delta.y() + (mapCreator.getMap()->getHeight() - y), 1, 1);
                }
            }
        }
    }
    
    void QMapWidget::paintRobot(QPainter& p_painter)
    {
        
        p_painter.setOpacity(1.0);
        
        RAIILock lock1(&positionMutex);
        
        if(positionCounter == 0)
            lastPositions.append(QPoint(robotPos.point.x * 100, robotPos.point.y * 100));
        positionCounter++;
        positionCounter %= 30;
        
        RAIILock lock2(&mapMutex);
        
        QPoint delta;
        delta.setX((width() - mapCreator.getMap()->getWidth()) / 2);
        delta.setY((height() - mapCreator.getMap()->getHeight()) / 2);
        
        int x, y, i;

        p_painter.setPen(Qt::black);
        p_painter.setBrush(QBrush(Qt::white));
        
        for(i = 0; i < lastPositions.size(); ++i)
        {
            x = delta.x() + lastPositions[i].x() + mapCreator.getXOffset();
            y = delta.y() + mapCreator.getMap()->getHeight() - (lastPositions[i].y() + mapCreator.getYOffset());
            p_painter.drawRect(x - 1, y - 1, 3, 3);
        }
        
        p_painter.setPen(Qt::blue);
        p_painter.setBrush(QBrush(Qt::green));
        
        x = delta.x() + robotPos.point.x * 100 + mapCreator.getXOffset();
        y = robotPos.point.y * 100 + mapCreator.getYOffset();
        
        y = delta.y() + mapCreator.getMap()->getHeight() - y;
        
        p_painter.drawRect(x - 1, y - 1, 3, 3);
    }
    
    void QMapWidget::setPosition(const RobotPosition& p_robotPos)
    {
        setRobotPosition(p_robotPos);
        setMapCreatorPosition(p_robotPos);
    }
    
    void QMapWidget::setRobotPosition(const RobotPosition& p_robotPos)
    {
        RAIILock lock(&positionMutex);
        robotPos = p_robotPos;
    }
    
    void QMapWidget::setMapCreatorPosition(const RobotPosition& p_robotPos)
    {
        RAIILock lock(&mapMutex);
        mapCreator.setPosition(p_robotPos);
    }
    
    void QMapWidget::step(const int p_id, const int p_distance)
    {
        RAIILock lock(&mapMutex);
        mapCreator.step(p_id, p_distance);
    }
}

#include <QSize>
#include "nxt_qt/QMapWidget.hpp"
#include "nxt_qt/Lock.hpp"

#define MAX_FIELD_VALUE 50

namespace minotaur
{
    QMapWidget::QMapWidget(QWidget *parent)
    : QWidget(parent)
    {
        setBackgroundRole(QPalette::Base);
        setAutoFillBackground(true);
        
        setMinimumSize(QSize(mapCreator.getMap()->getWidth(), mapCreator.getMap()->getHeight()));
    }
    
    void QMapWidget::paintEvent(QPaintEvent *event)
    {
        QPainter painter(this);
        QPoint delta;
        
        delta.setX((width() - mapCreator.getMap()->getWidth()) / 2);
        delta.setY((height() - mapCreator.getMap()->getHeight()) / 2);
        
        painter.setBrush(QBrush(Qt::white));
        painter.drawRect(0, 0, width() - 1, height() - 1);
        
        paintMap(painter, delta);
        paintRobot(painter, delta);
    }
    
    void QMapWidget::paintMap(QPainter& p_painter, QPoint& p_delta)
    {
        Lock lock(mapMutex);
        
        p_painter.setBrush(QBrush(Qt::red));
        for(int x = 0; x < mapCreator.getMap()->getWidth(); ++x)
        {
            for(int y = 0; y < mapCreator.getMap()->getHeight(); ++y)
            {
                float val = mapCreator.getMap()->getField()[y][x];
                if(val > MAX_FIELD_VALUE)
                    val = MAX_FIELD_VALUE;
                val = val / MAX_FIELD_VALUE;
                if(val > 0)
                {
                    p_painter.setOpacity(val);
                    p_painter.drawRect(p_delta.x() + x, p_delta.y() + (mapCreator.getMap()->getHeight() - y), 1, 1);
                }
            }
        }
    }
    
    void QMapWidget::paintRobot(QPainter& p_painter, QPoint& p_delta)
    {
        p_painter.setBrush(QBrush(Qt::blue));
        p_painter.setOpacity(1.0);
        
        Lock lock1(positionMutex);
        Lock lock2(mapMutex);
        
        int x = p_delta.x() + robotPos.point.x * 100 + mapCreator.getXOffset();
        int y = robotPos.point.y * 100 + mapCreator.getYOffset();
        
        y = p_delta.y() + mapCreator.getMap()->getHeight() - y;
        
        p_painter.drawRect(x - 1, y - 1, 3, 3);
    }
    
    void QMapWidget::setPosition(const RobotPosition& p_robotPos)
    {
        setRobotPosition(p_robotPos);
        setMapCreatorPosition(p_robotPos);
    }
    
    void QMapWidget::setRobotPosition(const RobotPosition& p_robotPos)
    {
        Lock lock(positionMutex);
        robotPos = p_robotPos;
    }
    
    void QMapWidget::setMapCreatorPosition(const RobotPosition& p_robotPos)
    {
        Lock lock(mapMutex);
        mapCreator.setPosition(p_robotPos);
    }
    
    void QMapWidget::step(const int p_id, const int p_distance)
    {
        Lock lock(mapMutex);
        mapCreator.step(p_id, p_distance);
    }
}
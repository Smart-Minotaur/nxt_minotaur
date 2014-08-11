#ifndef NXT_QT_QMAP_WIDGET_HPP_
#define NXT_QT_QMAP_WIDGET_HPP_

#include <pthread.h>
#include <QWidget>
#include <QPainter>
#include <QPoint>
#include <QVector>
#include "minotaur_map/MapCreator.hpp"
#include "minotaur_map/RobotPosition.hpp"

namespace minotaur
{
    class QMapWidget : public QWidget
    {
        
    private:
        MapCreator mapCreator;
        RobotPosition robotPos;
        pthread_mutex_t mapMutex;
        pthread_mutex_t positionMutex;
        QVector<QPoint> lastPositions;
        int positionCounter;
        
        void setRobotPosition(const RobotPosition& p_robotPos);
        void setMapCreatorPosition(const RobotPosition& p_robotPos);
        
    protected:
        void paintEvent(QPaintEvent *event);
        void paintMap(QPainter& p_painter);
        void paintRobot(QPainter& p_painter);
        
    public:
        QMapWidget(QWidget *parent);
        ~QMapWidget();
        
        void setPosition(const RobotPosition& p_robotPos);
        void step(const int p_id, const int p_distance);
    };
}

#endif

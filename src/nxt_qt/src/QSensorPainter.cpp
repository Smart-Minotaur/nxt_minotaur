#include "nxt_qt/QSensorPainter.hpp"
#include <QPainter>
#include <cmath>

#define SIZE_FACTOR 8.0f
#define MAX_RANGE 30

namespace minotaur
{
    QSensorPainter::QSensorPainter(QWidget *parent)
     : QWidget(parent)
    {
        setBackgroundRole(QPalette::Base);
        setAutoFillBackground(true);
    }
    
    QSize QSensorPainter::minimumSizeHint() const
    {
        return QSize(400, 200);
    }
    
    QSize QSensorPainter::sizeHint() const
    {
        return QSize(400, 200);
    }
    
    void QSensorPainter::addMeasurement(const QPoint& p_point)
    {
        sensorMeasurements.push_back(p_point);
    }
    
    int QSensorPainter::getCount()
    {
        return sensorMeasurements.size();
    }
    
    void QSensorPainter::setMeasurement(int p_idx, const QPoint& p_point)
    {
        sensorMeasurements[p_idx] = p_point;
    }
    
    void QSensorPainter::paintEvent(QPaintEvent *event)
    {
        int i;
        QPoint center(width() / 2, height() / 2);
        std::vector<QLine> lines;
        int radius = (height() / 30) + 1;
        
        QPainter painter(this);
        
        painter.setBrush(QBrush(Qt::white));
        painter.drawRect(0, 0, width(), height());
        
        painter.setBrush(QBrush(Qt::blue));
        
        painter.drawEllipse(center, radius, radius);
        
        painter.setBrush(QBrush(Qt::red));
        for(i = 0; i < sensorMeasurements.size(); ++i)
        {
            if( (sensorMeasurements[i].x() * sensorMeasurements[i].x() + sensorMeasurements[i].y() * sensorMeasurements[i].y()) < MAX_RANGE * MAX_RANGE)
                painter.drawEllipse(QPoint(center.x() + SIZE_FACTOR * sensorMeasurements[i].x(), center.y() + SIZE_FACTOR * sensorMeasurements[i].y()), radius, radius);
        }
        
    }
}
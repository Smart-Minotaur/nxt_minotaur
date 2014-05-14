#include "nxt_qt/QSensorPainter.hpp"
#include <QPainter>
#include <cmath>

#define SIZE_FACTOR 10.0f

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
        int radius = (width() / 30) + 1;
        
        for(i = 0; i < sensorMeasurements.size(); ++i)
        {
            QPoint tmpPoint(center.x() + sensorMeasurements[i].x(), center.y() + sensorMeasurements[i].y());
            float length = SIZE_FACTOR * sqrt(sensorMeasurements[i].x() * sensorMeasurements[i].x() + sensorMeasurements[i].y() * sensorMeasurements[i].y());
            float dx = -sensorMeasurements[i].x() / length;
            float dy = sensorMeasurements[i].x() / length;
            QLine tmpLine(tmpPoint.x() + dx * radius, tmpPoint.y() + dy * radius, tmpPoint.x() - dx * radius, tmpPoint.y() - dy * radius);
            lines.push_back(tmpLine);
        }
        
        QPainter painter(this);
        
        painter.setBrush(QBrush(Qt::white));
        painter.drawRect(0, 0, width(), height());
        
        painter.setBrush(QBrush(Qt::blue));
        
        painter.drawEllipse(center, radius, radius);
        
        for(i = 0; i < lines.size(); ++i)
            painter.drawLine(lines[i]);
        
    }
}
#ifndef NXT_QT_QSENSOR_PAINTER_HPP_
#define NXT_QT_QSENSOR_PAINTER_HPP_

#include <QWidget>
#include <QPoint>
#include <QSize>

namespace minotaur
{
    /**
     * \brief The QSensorPainter class is a simple QWdiget on which
     *        obstacles can be drawn, which are recognized by the 
     *        Ultrasonic sensors.
     */
    class QSensorPainter : public QWidget
    {
    private:
        std::vector<QPoint> sensorMeasurements;
        
    public:
        
        QSensorPainter(QWidget *parent);
        virtual ~QSensorPainter() { }
        
        void addMeasurement(const QPoint& p_point);
        int getCount();
        void setMeasurement(int p_idx, const QPoint& p_point);
        
        QSize minimumSizeHint() const;
        QSize sizeHint() const;
        
    protected:
        void paintEvent(QPaintEvent *event);
        
    };
}

#endif

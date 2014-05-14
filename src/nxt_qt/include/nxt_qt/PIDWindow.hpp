#ifndef PID_WINDOW_HPP_
#define PID_WINDOW_HPP_

#include <QtGui/QMainWindow>
#include <QPoint>
#include <qwt_plot_curve.h>
#include <qwt_plot_layout.h>
#include "nxt_qt/QSensorPainter.hpp"
#include "ui_pid_window.h"
#include "nxt_qt/QPIDNode.hpp"

#define SAMPLE_RANGE 400

namespace minotaur {
    
    class PIDWindow : public QMainWindow, public Ui::PIDControlWindow
    {
        Q_OBJECT
    private:
        QPIDNode pidNode;
        QSensorPainter *sensorPainter;
        
        int sampleCount;
        QwtPlotCurve linCurve, angCurve;
        double xSamples[SAMPLE_RANGE];
        double linSamples[SAMPLE_RANGE];
        double angSamples[SAMPLE_RANGE];
        
        void initComponents();
        void initPlotComponents();
        
        float getLinearVel();
        float getAngVel();
        
        float getKP();
        float getKI();
        float getKD();
        
        void updateMeasuredProgressBars(const QRobotVelocity& p_velocity);
        void updateMeasuredPlot(const QRobotVelocity& p_velocity);
        void updateProgressBarColor(QProgressBar *p_progressbar);
        
    private Q_SLOTS:
        void brake();
        void setPID();
        void updatePIDValues();
        void setVelocity();
        void updateVelocityValues();
        void processMeasuredVelocity(const QRobotVelocity p_velocity);
        void processMeasuredSensor(const QUltraSensor p_sensor);
    public:
        PIDWindow(QWidget *parent = 0);
        ~PIDWindow();
        
        QPIDNode& getPIDNode();
    };
    
}

#endif

#ifndef PID_WINDOW_HPP_
#define PID_WINDOW_HPP_

#include <QtGui/QMainWindow>
#include "ui_pid_window.h"
#include "nxt_qt/QPIDNode.hpp"
#include <qwt_plot_curve.h>
#include <qwt_plot_layout.h>

#define SAMPLE_RANGE 400

namespace minotaur {
    
    class PIDWindow : public QMainWindow, public Ui::PIDControlWindow
    {
        Q_OBJECT
    private:
        QPIDNode pidNode;
        
        int sampleCount;
        QwtPlotCurve leftCurve, rightCurve;
        double xSamples[SAMPLE_RANGE];
        double leftSamples[SAMPLE_RANGE];
        double rightSamples[SAMPLE_RANGE];
        
        void initComponents();
        void initPlotComponents();
        
        float getLinearVel();
        float getAngVel();
        
        float getKP();
        float getKI();
        float getKD();
        
        void updateMeasuredProgressBars(const QMotorVelocity& p_velocity);
        void updateMeasuredPlot(const QMotorVelocity& p_velocity);
        void updateProgressBarColor(QProgressBar *p_progressbar);
        
    private Q_SLOTS:
        void brake();
        void setPID();
        void updatePIDValues();
        void setVelocity();
        void updateVelocityValues();
        void processTargetMotorVelocity(const QMotorVelocity p_velocity);
        void processMeasuredMotorVelocity(const QMotorVelocity p_velocity);
        void setInitModel();
    public:
        PIDWindow(QWidget *parent = 0);
        ~PIDWindow();
        
        QPIDNode& getPIDNode();
    };
    
}

#endif

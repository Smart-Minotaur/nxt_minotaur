#ifndef PID_WINDOW_HPP_
#define PID_WINDOW_HPP_

#include <QtGui/QMainWindow>
#include "ui_pid_window.h"
#include "nxt_qt/QPIDNode.hpp"

namespace minotaur {
    
    class PIDWindow : public QMainWindow, public Ui::PIDControlWindow
    {
        Q_OBJECT
    private:
        QPIDNode pidNode;
        
        void initComponents();
        
        float getLinearVel();
        float getAngVel();
        
        float getKP();
        float getKI();
        float getKD();
        
        void updateProgressBarColor(QProgressBar *p_progressbar);
        
    private Q_SLOTS:
        void brake();
        void setPID();
        void updatePIDValues();
        void setVelocity();
        void updateVelocityValues();
        void processTargetMotorVelocity(const QMotorVelocity p_velocity);
        void processMeasuredMotorVelocity(const QMotorVelocity p_velocity);
    public:
        PIDWindow(QWidget *parent = 0);
        ~PIDWindow();
        
        QPIDNode& getPIDNode();
    };
    
}

#endif

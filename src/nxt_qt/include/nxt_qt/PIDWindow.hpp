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
        
    private Q_SLOTS:
        void brake();
        void setPID();
        void updatePIDValues();
        void setVelocity();
        void updateVelocityValues();
        void processTargetMotorVelocity(const nxt_beagle::MVelocity& p_msg);
        void processMeasuredMotorVelocity(const nxt_beagle::MVelocity& p_msg);
    public:
        PIDWindow(QWidget *parent = 0);
        ~PIDWindow();
        
        QPIDNode& getPIDNode();
    };
    
}

#endif

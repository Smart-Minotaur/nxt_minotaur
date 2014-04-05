#include <cstdio>
#include "nxt_qt/PIDWindow.hpp"

#define MAX_LIN_VEL 0.5f
#define MAX_ANG_VEL 0.5f

#define MAX_KP 2.0f
#define MAX_KI 2.0f
#define MAX_KD 2.0f
#define DEF_KP 0.5f
#define DEF_KI 0.3f
#define DEF_KD 0.1f

using namespace Qt;

namespace minotaur
{
    
    
    PIDWindow::PIDWindow(QWidget *parent)
    :QMainWindow(parent)
    {
        setupUi(this);
        initComponents();
        
        connect(&pidNode, SIGNAL(targetMotorVelocityUpdated(const nxt_beagle::MVelocity&)), this, SLOT(processTargetMotorVelocity(const nxt_beagle::MVelocity&)));
        connect(&pidNode, SIGNAL(measuredMotorVelocityUpdated(const nxt_beagle::MVelocity&)), this, SLOT(processMeasuredMotorVelocity(const nxt_beagle::MVelocity&)));
        
        connect(BrakeButton, SIGNAL(clicked()), this, SLOT(brake()));
        
        connect(VelSlider, SIGNAL(valueChanged(int)), this, SLOT(updateVelocityValues()));
        connect(AngSlider, SIGNAL(valueChanged(int)), this, SLOT(updateVelocityValues()));
        connect(VelSlider, SIGNAL(sliderReleased()), this, SLOT(setVelocity()));
        connect(AngSlider, SIGNAL(sliderReleased()), this, SLOT(setVelocity()));
        
        connect(KPSlider, SIGNAL(valueChanged(int)), this, SLOT(updatePIDValues()));
        connect(KISlider, SIGNAL(valueChanged(int)), this, SLOT(updatePIDValues()));
        connect(KDSlider, SIGNAL(valueChanged(int)), this, SLOT(updatePIDValues()));
        connect(KPSlider, SIGNAL(sliderReleased()), this, SLOT(setPID()));
        connect(KISlider, SIGNAL(sliderReleased()), this, SLOT(setPID()));
        connect(KDSlider, SIGNAL(sliderReleased()), this, SLOT(setPID()));
    }
    
    PIDWindow::~PIDWindow() { }
    
    void PIDWindow::initComponents()
    {
        VelSlider->setRange(-100, 100);
        VelSlider->setValue(0);
        
        AngSlider->setRange(-100, 100);
        AngSlider->setValue(0);
        
        KPSlider->setRange(0, 100);
        KPSlider->setValue((DEF_KP / MAX_KP) * 100);
        
        KISlider->setRange(0, 100);
        KISlider->setValue((DEF_KD / MAX_KD) * 100);
        
        KDSlider->setRange(0, 100);
        KDSlider->setValue((DEF_KD / MAX_KD) * 100);
    }
    
    QPIDNode& PIDWindow::getPIDNode()
    {
        return pidNode;
    }
    
    void PIDWindow::brake()
    {
        VelSlider->setValue(0);
        AngSlider->setValue(0);
        setVelocity();
    }
    
    void PIDWindow::setVelocity()
    {
        updateVelocityValues();
        pidNode.setRobotVelocity(getLinearVel(), getAngVel());
    }
    
    void PIDWindow::updateVelocityValues()
    {
        VelValueLabel->setText(QString::number(getLinearVel(), 'f', 2));
        AngValueLabel->setText(QString::number(getAngVel(), 'f', 2));
    }
    
    float PIDWindow::getLinearVel()
    {
        return ((float) (VelSlider->value()) / 100.0f) * MAX_LIN_VEL;
    }
    
    float PIDWindow::getAngVel()
    {
        return ((float) (AngSlider->value()) / 100.0f) * MAX_ANG_VEL;
    }
    
    void PIDWindow::setPID()
    {
        updatePIDValues();
        pidNode.setPIDParameter(getKP(), getKI(), getKD());
    }
    
    void PIDWindow::updatePIDValues()
    {
        KPValueLabel->setText(QString::number(getKP(), 'f', 2));
        KIValueLabel->setText(QString::number(getKI(), 'f', 2));
        KDValueLabel->setText(QString::number(getKD(), 'f', 2));
    }
    
    float PIDWindow::getKP()
    {
        return ((float) (KPSlider->value()) / 100.0f) * MAX_KP;
    }
    
    float PIDWindow::getKI()
    {
        return ((float) (KISlider->value()) / 100.0f) * MAX_KI;
    }
    
    float PIDWindow::getKD()
    {
        return ((float) (KDSlider->value()) / 100.0f) * MAX_KD;
    }
    
    void PIDWindow::processTargetMotorVelocity(const nxt_beagle::MVelocity& p_msg)
    {
        LeftTargetProgressBar->setValue( (p_msg.leftVelocity / MAX_LIN_VEL) * 100);
        LeftTargetValueLabel->setText(QString::number(p_msg.leftVelocity, 'f', 2));
        
        RightTargetProgressBar->setValue( (p_msg.rightVelocity / MAX_LIN_VEL) * 100);
        RightTargetValueLabel->setText(QString::number(p_msg.rightVelocity, 'f', 2));
    }
    
    void PIDWindow::processMeasuredMotorVelocity(const nxt_beagle::MVelocity& p_msg)
    {
        int value = (p_msg.leftVelocity / MAX_LIN_VEL) * 100;
        LeftTargetProgressBar->setValue(value);
        LeftMeasuredValueLabel->setText(QString::number(p_msg.leftVelocity, 'f', 2));
        
        value = (p_msg.rightVelocity / MAX_LIN_VEL) * 100;
        RightTargetProgressBar->setValue(value);
        RightMeasuredValueLabel->setText(QString::number(p_msg.rightVelocity, 'f', 2));
    }
    
}
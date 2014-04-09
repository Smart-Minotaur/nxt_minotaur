/*
 * Author: Fabian Meyer
 */

#include "nxt_beagle/Config.hpp"
#include <cstdio>
#include "nxt_qt/PIDWindow.hpp"

#define MAX_LIN_VEL 0.5f
#define MAX_ANG_VEL 2.0f
#define MAX_PLOT_VEL 0.3

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
        
        connect(&pidNode, SIGNAL(targetMotorVelocityUpdated(const QMotorVelocity)), this, SLOT(processTargetMotorVelocity(const QMotorVelocity)));
        connect(&pidNode, SIGNAL(measuredMotorVelocityUpdated(const QMotorVelocity)), this, SLOT(processMeasuredMotorVelocity(const QMotorVelocity)));
        
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
        
        initComponents();
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
        
        LeftTargetProgressBar->setProperty("defaultStyleSheet", LeftTargetProgressBar->styleSheet());
        RightTargetProgressBar->setProperty("defaultStyleSheet", RightTargetProgressBar->styleSheet());
        LeftMeasuredProgressBar->setProperty("defaultStyleSheet", LeftMeasuredProgressBar->styleSheet());
        RightMeasuredProgressBar->setProperty("defaultStyleSheet", RightMeasuredProgressBar->styleSheet());
        
        initPlotComponents();
    }

    void PIDWindow::initPlotComponents()
    {
        memset(leftSamples, 0, SAMPLE_RANGE * sizeof (double));
        memset(rightSamples, 0, SAMPLE_RANGE * sizeof (double)); 
        for(int i = 0; i < SAMPLE_RANGE; i++)
            xSamples[i] = i;
        
        sampleCount = 0;
        
        leftCurve.setPen(QColor(Qt::red));
        leftCurve.setSamples(xSamples, leftSamples, SAMPLE_RANGE);
        rightCurve.setPen(QColor(Qt::green));
        rightCurve.setSamples(xSamples, rightSamples, SAMPLE_RANGE);
        
        leftCurve.attach(MeasureLeftPlot);
        rightCurve.attach(MeasureRightPlot);
        
        MeasureLeftPlot->setTitle("Left Motor");
        MeasureLeftPlot->setCanvasBackground(QColor(Qt::white));
        MeasureLeftPlot->setAxisScale(QwtPlot::xBottom, 0, SAMPLE_RANGE, 50);
        MeasureLeftPlot->setAxisScale(QwtPlot::yLeft, -MAX_PLOT_VEL, MAX_PLOT_VEL, 0.05);
        MeasureLeftPlot->setAxisTitle(QwtPlot::xBottom, "step");
        MeasureLeftPlot->setAxisTitle(QwtPlot::yLeft, "m/s");
        
        MeasureRightPlot->setTitle("Right Motor");
        MeasureRightPlot->setCanvasBackground(QColor(Qt::white));
        MeasureRightPlot->setAxisScale(QwtPlot::xBottom, 0, SAMPLE_RANGE, 50);
        MeasureRightPlot->setAxisScale(QwtPlot::yLeft, -MAX_PLOT_VEL, MAX_PLOT_VEL, 0.05);
        MeasureRightPlot->setAxisTitle(QwtPlot::xBottom, "step");
        MeasureRightPlot->setAxisTitle(QwtPlot::yLeft, "m/s");
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
    
    void PIDWindow::processTargetMotorVelocity(const QMotorVelocity p_velocity)
    {
        int value = abs((p_velocity.leftMPS / MAX_LIN_VEL) * 100);
        LeftTargetProgressBar->setValue(value);
        updateProgressBarColor(LeftTargetProgressBar);
        LeftTargetValueLabel->setText(QString::number(p_velocity.leftMPS, 'f', 2));
        
        value = abs((p_velocity.rightMPS / MAX_LIN_VEL) * 100);
        RightTargetProgressBar->setValue(value);
        updateProgressBarColor(RightTargetProgressBar);
        RightTargetValueLabel->setText(QString::number(p_velocity.rightMPS, 'f', 2));
    }
    
    void PIDWindow::processMeasuredMotorVelocity(const QMotorVelocity p_velocity)
    {
        updateMeasuredProgressBars(p_velocity);
        updateMeasuredPlot(p_velocity);
    }
    
    void PIDWindow::updateMeasuredProgressBars(const QMotorVelocity& p_velocity)
    {
        int value = abs((p_velocity.leftMPS / MAX_LIN_VEL) * 100);
        LeftMeasuredProgressBar->setValue(value);
        updateProgressBarColor(LeftMeasuredProgressBar);
        LeftMeasuredValueLabel->setText(QString::number(p_velocity.leftMPS, 'f', 2));
        
        value = abs((p_velocity.rightMPS / MAX_LIN_VEL) * 100);
        RightMeasuredProgressBar->setValue(value);
        updateProgressBarColor(RightMeasuredProgressBar);
        RightMeasuredValueLabel->setText(QString::number(p_velocity.rightMPS, 'f', 2));
    }
    
    void PIDWindow::updateMeasuredPlot(const QMotorVelocity& p_velocity)
    {
        leftSamples[sampleCount] = p_velocity.leftMPS;
        rightSamples[sampleCount] = p_velocity.rightMPS;
        
        leftCurve.setSamples(xSamples, leftSamples, SAMPLE_RANGE);
        rightCurve.setSamples(xSamples, rightSamples, SAMPLE_RANGE);
        
        MeasureLeftPlot->replot();
        MeasureRightPlot->replot();
        
        ++sampleCount;
        sampleCount %= SAMPLE_RANGE;
    }
    
    void PIDWindow::updateProgressBarColor(QProgressBar *p_progressbar)
    {
        if(p_progressbar->value() <= 50)
            p_progressbar->setStyleSheet(p_progressbar->property("defaultStyleSheet").toString() + " QProgressBar::chunk { background: green; }");
        else if(p_progressbar->value() <= 75)
            p_progressbar->setStyleSheet(p_progressbar->property("defaultStyleSheet").toString() + " QProgressBar::chunk { background: yellow; }");
        else
            p_progressbar->setStyleSheet(p_progressbar->property("defaultStyleSheet").toString() + " QProgressBar::chunk { background: red; }");
    }
    
    void PIDWindow::setInitModel()
    {
        pidNode.setModel(HERACLES_NAME);
    }
    
}
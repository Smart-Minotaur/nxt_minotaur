#include "nxt_qt/MouseMonitorWindow.hpp"

#include <QWidget>
#include <QMessageBox>
#include <QToolBar>
#include <QToolButton>
#include <string>
#include <iostream>

#include "ros/ros.h"

#define DEFAULT_SAMPLE_RATE_MS 1000

// Plots
#define MAX_Y_RANGE_DISP 2
#define Y_STEP_DISP MAX_Y_RANGE_DISP / 10
#define X_STEP_DISP SAMPLE_RANGE / 10

#define MAX_Y_RANGE_SPEED 100
#define Y_STEP_SPEED MAX_Y_RANGE_SPEED / 5
#define X_STEP_SPEED SAMPLE_RANGE / 10

namespace minotaur
{

    MouseMonitorWindow::MouseMonitorWindow(QWidget *parent) :
        QMainWindow(parent)
    {
        setupUi(this);

        initWidgets();
        initTable();
        initPlots();
        initTimer();
        initToolbar();

        connectSlots();

        timer->start(sampleRateMs);
    }

    MouseMonitorWindow::~MouseMonitorWindow()
    {
        delete directionWidget1;
        delete directionWidget2;
        delete pathWidget;
        delete timer;
    }

    void MouseMonitorWindow::connectSlots()
    {
        connect(&monitorNode, SIGNAL(measuredMouseData(const MouseData)),
                this, SLOT(processMouseData(const MouseData)));

        connect(&monitorNode, SIGNAL(measuredMouseSettings(const pln_minotaur::PLN2033_Settings)),
                this, SLOT(processMouseSettings(const pln_minotaur::PLN2033_Settings)));

        connect(zoomSlider, SIGNAL(valueChanged(int)),
                pathWidget, SLOT(zoomValueChanged(const int)));

        connect(trackSensor1, SIGNAL(stateChanged(int)),
                pathWidget, SLOT(sensor1Enable(const int)));
        connect(trackSensor2, SIGNAL(stateChanged(int)),
                pathWidget, SLOT(sensor2Enable(const int)));

        connect(actionAbout, SIGNAL(triggered()), this, SLOT(openAboutWindow()));

        connect(getSensorSettingsBtn, SIGNAL(clicked()), this, SLOT(getSensorSettingsBtnClicked()));

        connect(detailDebugging, SIGNAL(stateChanged(int)), this, SLOT(detailDebuggingEnable(const int)));
    }

    void MouseMonitorWindow::initTimer()
    {
        sampleRateMs = DEFAULT_SAMPLE_RATE_MS;

        timer = new QTimer(this);
        connect(timer, SIGNAL(timeout()), this, SLOT(timerTimeout()));
    }

    void MouseMonitorWindow::initToolbar()
    {
        toolBar->setMovable(false);

        sampleRateBtn = new QToolButton(this);
        sampleRateBtn->setText("Set Sample Rate");
        sampleRateBtn->setArrowType(Qt::DownArrow);
        sampleRateBtn->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);

        resolution1Btn = new QToolButton(this);
        resolution1Btn->setText("Set resolution sensor 1");
        resolution1Btn->setArrowType(Qt::DownArrow);
        resolution1Btn->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);

        resolution2Btn = new QToolButton(this);
        resolution2Btn->setText("Set resolution sensor 2");
        resolution2Btn->setArrowType(Qt::DownArrow);
        resolution2Btn->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);

        sampleRateEdit = new QLineEdit(this);
        sampleRateEdit->setText(QString("%1").arg(sampleRateMs));

        resolution1Edit = new QLineEdit(this);
        resolution1Edit->setText("Get sensor settings");

        resolution2Edit = new QLineEdit(this);
        resolution2Edit->setText("Get sensor settings");

        toolBar->addWidget(sampleRateBtn);
        toolBar->addWidget(sampleRateEdit);
        toolBar->addWidget(resolution1Btn);
        toolBar->addWidget(resolution1Edit);
        toolBar->addWidget(resolution2Btn);
        toolBar->addWidget(resolution2Edit);

        connect(sampleRateBtn, SIGNAL(clicked()), this, SLOT(sampleRateBtnClicked()));
        connect(resolution1Btn, SIGNAL(clicked()), this, SLOT(resolution1BtnClicked()));
        connect(resolution2Btn, SIGNAL(clicked()), this, SLOT(resolution2BtnClicked()));
    }

    void MouseMonitorWindow::initWidgets()
    {
        directionWidget1 = new DirectionWidget(directionFrame1);
        directionFrame1->layout()->addWidget(directionWidget1);

        directionWidget2 = new DirectionWidget(directionFrame2);
        directionFrame2->layout()->addWidget(directionWidget2);

        pathWidget = new TrackPathWidget(trackPathFrame);
        trackPathFrame->layout()->addWidget(pathWidget);
        pathWidget->init();
    }

    void MouseMonitorWindow::initTable()
    {
        sensorSettingsTable->setColumnCount(13);
        sensorSettingsTable->setRowCount(2);

        QStringList headerLabels;
        headerLabels << "Status Register" << "Delta X Disp" << "Delta Y Disp"
                     << "Command High" << "Command Low" << "Memory pointer" << "Memory Data"
                     << "Mode Control" << "Power Control" << "Mode Status" << "System Control"
                     << "Misc" << "Interrupt Output";

        sensorSettingsTable->setHorizontalHeaderLabels(headerLabels);
        sensorSettingsTable->setEditTriggers(QAbstractItemView::NoEditTriggers);
        sensorSettingsTable->horizontalHeader()->setResizeMode(QHeaderView::ResizeToContents);
    }

    void MouseMonitorWindow::initPlots()
    {
        memset(xDisp1, 0, SAMPLE_RANGE * sizeof(double));
        memset(yDisp1, 0, SAMPLE_RANGE * sizeof(double));
        memset(xDisp2, 0, SAMPLE_RANGE * sizeof(double));
        memset(yDisp2, 0, SAMPLE_RANGE * sizeof(double));

        memset(xSpeed1, 0, SAMPLE_RANGE * sizeof(double));
        memset(ySpeed1, 0, SAMPLE_RANGE * sizeof(double));
        memset(xSpeed2, 0, SAMPLE_RANGE * sizeof(double));
        memset(ySpeed2, 0, SAMPLE_RANGE * sizeof(double));

        for(int i = 0; i < SAMPLE_RANGE; ++i) {
            xSamplesSensor1[i] = i;
            xSamplesSensor2[i] = i;
        }

        sampleCountSensor1 = 0;
        sampleCountSensor2 = 0;

        // Displacement
        xDisp1Curve.setPen(QColor(Qt::blue));
        xDisp1Curve.setSamples(xSamplesSensor1, xDisp1, SAMPLE_RANGE);
        xDisp1Curve.attach(x_disp1_viz);
        initPlot(x_disp1_viz, "X Displacement", MAX_Y_RANGE_DISP, X_STEP_DISP, Y_STEP_DISP, "step", "cm");

        yDisp1Curve.setPen(QColor(Qt::blue));
        yDisp1Curve.setSamples(xSamplesSensor1, yDisp1, SAMPLE_RANGE);
        yDisp1Curve.attach(y_disp1_viz);
        initPlot(y_disp1_viz, "Y Displacement", MAX_Y_RANGE_DISP, X_STEP_DISP, Y_STEP_DISP, "step", "cm");

        xDisp2Curve.setPen(QColor(Qt::red));
        xDisp2Curve.setSamples(xSamplesSensor2, xDisp2, SAMPLE_RANGE);
        xDisp2Curve.attach(x_disp2_viz);
        initPlot(x_disp2_viz, "X Displacement", MAX_Y_RANGE_DISP, X_STEP_DISP, Y_STEP_DISP, "step", "cm");

        yDisp2Curve.setPen(QColor(Qt::red));
        yDisp2Curve.setSamples(xSamplesSensor2, yDisp2, SAMPLE_RANGE);
        yDisp2Curve.attach(y_disp2_viz);
        initPlot(y_disp2_viz, "Y Displacement", MAX_Y_RANGE_DISP, X_STEP_DISP, Y_STEP_DISP, "step", "cm");

        // Speed
        xSpeed1Curve.setPen(QColor(Qt::blue));
        xSpeed1Curve.setSamples(xSamplesSensor1, xSpeed1, SAMPLE_RANGE);
        xSpeed1Curve.attach(x_speed1_viz);
        initPlot(x_speed1_viz, "X Speed", MAX_Y_RANGE_SPEED, X_STEP_SPEED, Y_STEP_SPEED, "step", "cm/s");

        ySpeed1Curve.setPen(QColor(Qt::blue));
        ySpeed1Curve.setSamples(xSamplesSensor1, ySpeed1, SAMPLE_RANGE);
        ySpeed1Curve.attach(y_speed1_viz);
        initPlot(y_speed1_viz, "Y Speed", MAX_Y_RANGE_SPEED, X_STEP_SPEED, Y_STEP_SPEED, "step", "cm/s");

        xSpeed2Curve.setPen(QColor(Qt::red));
        xSpeed2Curve.setSamples(xSamplesSensor2, xSpeed2, SAMPLE_RANGE);
        xSpeed2Curve.attach(x_speed2_viz);
        initPlot(x_speed2_viz, "X Speed", MAX_Y_RANGE_SPEED, X_STEP_SPEED, Y_STEP_SPEED, "step", "cm/s");

        ySpeed2Curve.setPen(QColor(Qt::red));
        ySpeed2Curve.setSamples(xSamplesSensor2, ySpeed2, SAMPLE_RANGE);
        ySpeed2Curve.attach(y_speed2_viz);
        initPlot(y_speed2_viz, "Y Speed", MAX_Y_RANGE_SPEED, X_STEP_SPEED, Y_STEP_SPEED, "step", "cm/s");
    }

    void MouseMonitorWindow::initPlot(QwtPlot *plot,
                                      QString title,
                                      double maxYRange,
                                      double xStep,
                                      double yStep,
                                      QString xAxisTitle,
                                      QString yAxisTitle)
    {
        plot->setTitle(title);
        plot->setAxisScale(QwtPlot::xBottom, 0, SAMPLE_RANGE, xStep);
        plot->setAxisScale(QwtPlot::yLeft, -maxYRange, maxYRange, yStep);
        plot->setAxisTitle(QwtPlot::xBottom, xAxisTitle);
        plot->setAxisTitle(QwtPlot::yLeft, yAxisTitle);
    }

    void MouseMonitorWindow::updatePlotSensor1(MouseData data)
    {
        // Displacement plot
        xDisp1[sampleCountSensor1] = data.x_disp;
        yDisp1[sampleCountSensor1] = data.y_disp;

        xDisp1Curve.setSamples(xSamplesSensor1, xDisp1, SAMPLE_RANGE);
        yDisp1Curve.setSamples(xSamplesSensor1, yDisp1, SAMPLE_RANGE);

        x_disp1_viz->replot();
        y_disp1_viz->replot();

        // Speed plot
        xSpeed1[sampleCountSensor1] = data.x_speed;
        ySpeed1[sampleCountSensor1] = data.y_speed;

        xSpeed1Curve.setSamples(xSamplesSensor1, xSpeed1, SAMPLE_RANGE);
        ySpeed1Curve.setSamples(xSamplesSensor1, ySpeed1, SAMPLE_RANGE);

        x_speed1_viz->replot();
        y_speed1_viz->replot();

        ++sampleCountSensor1;
        sampleCountSensor1 %= SAMPLE_RANGE;
    }

    void MouseMonitorWindow::updatePlotSensor2(MouseData data)
    {
        // Displacement plot
        xDisp2[sampleCountSensor2] = data.x_disp;
        yDisp2[sampleCountSensor2] = data.y_disp;

        xDisp2Curve.setSamples(xSamplesSensor2, xDisp2, SAMPLE_RANGE);
        yDisp2Curve.setSamples(xSamplesSensor2, yDisp2, SAMPLE_RANGE);

        x_disp2_viz->replot();
        y_disp2_viz->replot();

        // Speed plot
        xSpeed2[sampleCountSensor2] = data.x_speed;
        ySpeed2[sampleCountSensor2] = data.y_speed;

        xSpeed2Curve.setSamples(xSamplesSensor2, xSpeed2, SAMPLE_RANGE);
        ySpeed2Curve.setSamples(xSamplesSensor2, ySpeed2, SAMPLE_RANGE);

        x_speed2_viz->replot();
        y_speed2_viz->replot();

        ++sampleCountSensor2;
        sampleCountSensor2 %= SAMPLE_RANGE;
    }

    void MouseMonitorWindow::timerTimeout()
    {
        //processMouseData(monitorNode.getMouseData(SENSOR1));
        //processMouseData(monitorNode.getMouseData(SENSOR2));
    }

    MouseMonitorNode& MouseMonitorWindow::getMonitorNode()
    {
        return monitorNode;
    }

    void MouseMonitorWindow::processMouseData(const MouseData data)
    {
        if (data.id == SENSOR1) {
            name1->setText(QString::fromStdString(data.id));
            x_disp1->setText(QString("%1").arg(data.x_disp, 0, 'f', 8));
            y_disp1->setText(QString("%1").arg(data.y_disp, 0, 'f', 8));
            x_speed1->setText(QString("%1").arg(data.x_speed, 0, 'f', 8));
            y_speed1->setText(QString("%1").arg(data.y_speed, 0, 'f', 8));

            directionWidget1->updateWidget(data);
            pathWidget->updateWidget(data);
            updatePlotSensor1(data);
        } else if (data.id == SENSOR2) {
            name2->setText(QString::fromStdString(data.id));
            x_disp2->setText(QString("%1").arg(data.x_disp, 0, 'f', 8));
            y_disp2->setText(QString("%1").arg(data.y_disp, 0, 'f', 8));
            x_speed2->setText(QString("%1").arg(data.x_speed, 0, 'f', 8));
            y_speed2->setText(QString("%1").arg(data.y_speed, 0, 'f', 8));

            directionWidget2->updateWidget(data);
            pathWidget->updateWidget(data);
            updatePlotSensor2(data);
        }
    }

    void MouseMonitorWindow::processMouseSettings(const pln_minotaur::PLN2033_Settings settings)
    {
        if (settings.spiDevice == SENSOR1) {
            sensorSettingsTable->setItem(0, 0, new QTableWidgetItem(uintToQString(settings.status_register)));
            sensorSettingsTable->setItem(0, 1, new QTableWidgetItem(uintToQString(settings.delta_x_disp_register)));
            sensorSettingsTable->setItem(0, 2, new QTableWidgetItem(uintToQString(settings.delta_y_disp_register)));
            sensorSettingsTable->setItem(0, 3, new QTableWidgetItem(uintToQString(settings.command_high_register)));
            sensorSettingsTable->setItem(0, 4, new QTableWidgetItem(uintToQString(settings.command_low_register)));
            sensorSettingsTable->setItem(0, 5, new QTableWidgetItem(uintToQString(settings.memory_pointer_register)));
            sensorSettingsTable->setItem(0, 6, new QTableWidgetItem(uintToQString(settings.memory_data_register)));
            sensorSettingsTable->setItem(0, 7, new QTableWidgetItem(uintToQString(settings.mode_control_register)));
            sensorSettingsTable->setItem(0, 8, new QTableWidgetItem(uintToQString(settings.power_control_register)));
            sensorSettingsTable->setItem(0, 9, new QTableWidgetItem(uintToQString(settings.mode_status_register)));
            sensorSettingsTable->setItem(0, 10, new QTableWidgetItem(uintToQString(settings.system_control_register)));
            sensorSettingsTable->setItem(0, 11, new QTableWidgetItem(uintToQString(settings.miscellaneous_register)));
            sensorSettingsTable->setItem(0, 12, new QTableWidgetItem(uintToQString(settings.interrupt_output_register)));
        } else if (settings.spiDevice == SENSOR2) {
            sensorSettingsTable->setItem(1, 0, new QTableWidgetItem(uintToQString(settings.status_register)));
            sensorSettingsTable->setItem(1, 1, new QTableWidgetItem(uintToQString(settings.delta_x_disp_register)));
            sensorSettingsTable->setItem(1, 2, new QTableWidgetItem(uintToQString(settings.delta_y_disp_register)));
            sensorSettingsTable->setItem(1, 3, new QTableWidgetItem(uintToQString(settings.command_high_register)));
            sensorSettingsTable->setItem(1, 4, new QTableWidgetItem(uintToQString(settings.command_low_register)));
            sensorSettingsTable->setItem(1, 5, new QTableWidgetItem(uintToQString(settings.memory_pointer_register)));
            sensorSettingsTable->setItem(1, 6, new QTableWidgetItem(uintToQString(settings.memory_data_register)));
            sensorSettingsTable->setItem(1, 7, new QTableWidgetItem(uintToQString(settings.mode_control_register)));
            sensorSettingsTable->setItem(1, 8, new QTableWidgetItem(uintToQString(settings.power_control_register)));
            sensorSettingsTable->setItem(1, 9, new QTableWidgetItem(uintToQString(settings.mode_status_register)));
            sensorSettingsTable->setItem(1, 10, new QTableWidgetItem(uintToQString(settings.system_control_register)));
            sensorSettingsTable->setItem(1, 11, new QTableWidgetItem(uintToQString(settings.miscellaneous_register)));
            sensorSettingsTable->setItem(1, 12, new QTableWidgetItem(uintToQString(settings.interrupt_output_register)));
        }
    }

    void MouseMonitorWindow::openAboutWindow()
    {
        QMessageBox::about(this, tr("About MouseMonitor"),
                           tr("This application allows to monitor two Philips PLN2033 Sensors."));
    }

    void MouseMonitorWindow::sampleRateBtnClicked()
    {
        sampleRateMs = sampleRateEdit->text().toInt();
        timer->setInterval(sampleRateMs);
    }

    void MouseMonitorWindow::resolution1BtnClicked()
    {

    }

    void MouseMonitorWindow::resolution2BtnClicked()
    {

    }

    void MouseMonitorWindow::getSensorSettingsBtnClicked()
    {
        // TODO: X and Y resolution
        pln_minotaur::PLN2033_Settings settings1 = monitorNode.getMouseSettings(SENSOR1);
        if (settings1.spiDevice == SENSOR1) {
            resolution1Edit->setText(QString("%1").arg(settings1.getXResolution()));
            processMouseSettings(settings1);
        }

        pln_minotaur::PLN2033_Settings settings2 = monitorNode.getMouseSettings(SENSOR2);
        if (settings2.spiDevice == SENSOR2) {
            resolution2Edit->setText(QString("%1").arg(settings2.getXResolution()));
            processMouseSettings(settings2);
        }
    }

    void MouseMonitorWindow::detailDebuggingEnable(const int status)
    {
        if (status == 2)
            detailDebugginFrame->setEnabled(true);
        else
            detailDebugginFrame->setEnabled(false);
    }

    QString MouseMonitorWindow::uintToQString(uint data)
    {
        return QString("%1").arg(data, 4, 16, QChar('0'));
    }

}

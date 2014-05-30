#include "nxt_qt/MouseMonitorWindow.hpp"

#include <QWidget>
#include <QMessageBox>
#include <QToolBar>
#include <QToolButton>
#include <string>
#include <iostream>

#include "ros/ros.h"

#define DEFAULT_SAMPLE_RATE_MS 100

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
        initDetail();

        connectSlots();

        timer->start(sampleRateMs);
    }

    MouseMonitorWindow::~MouseMonitorWindow()
    {
        delete directionWidget1;
        delete directionWidget2;
        delete pathWidget;
        delete timer;

        delete xDisp1Plot;
        delete yDisp1Plot;
        delete xDisp2Plot;
        delete yDisp2Plot;
        delete xSpeed1Plot;
        delete ySpeed1Plot;
        delete xSpeed2Plot;
        delete ySpeed2Plot;

        delete detail1Plot;
        delete detail2Plot;
    }

    void MouseMonitorWindow::connectSlots()
    {
        // Track path tab
        connect(zoomSlider, SIGNAL(valueChanged(int)),
                pathWidget, SLOT(zoomValueChanged(const int)));

        connect(trackSensor1, SIGNAL(stateChanged(int)),
                pathWidget, SLOT(sensor1Enable(const int)));
        connect(trackSensor2, SIGNAL(stateChanged(int)),
                pathWidget, SLOT(sensor2Enable(const int)));

        connect(trackPathResetBtn, SIGNAL(clicked()), this, SLOT(trackPathResetBtnClicked()));

        // Menu bar
        connect(actionAbout, SIGNAL(triggered()), this, SLOT(openAboutWindow()));

        // Sensor data tab
        connect(getSensorSettingsBtn, SIGNAL(clicked()), this, SLOT(getSensorSettingsBtnClicked()));

        // Detail debugging tab
        connect(detailDebugging, SIGNAL(stateChanged(int)), this, SLOT(detailDebuggingEnable(const int)));
        connect(getData1Btn, SIGNAL(clicked()), this, SLOT(getData1BtnClicked()));
        connect(getData2Btn, SIGNAL(clicked()), this, SLOT(getData2BtnClicked()));
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
        directionWidget1 = new DirectionWidget(Qt::blue, directionFrame1);
        directionFrame1->layout()->addWidget(directionWidget1);

        directionWidget2 = new DirectionWidget(Qt::red, directionFrame2);
        directionFrame2->layout()->addWidget(directionWidget2);

        pathWidget = new TrackPathWidget(trackPathFrame);
        trackPathFrame->layout()->addWidget(pathWidget);
        pathWidget->init(20, 20);//trackPathFrame->width() / 2.0, trackPathFrame->height() / 2);

        x_disp1_abs->setText("0");
        y_disp1_abs->setText("0");
        x_disp2_abs->setText("0");
        y_disp2_abs->setText("0");
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
        xDisp1Plot = new MouseMonitorPlot();
        xDisp1Plot->init(Qt::blue, "X Displacement", "step", "cm");
        dispBox1->layout()->addWidget(xDisp1Plot);

        yDisp1Plot = new MouseMonitorPlot();
        yDisp1Plot->init(Qt::blue, "Y Displacement", "step", "cm");
        dispBox1->layout()->addWidget(yDisp1Plot);

        xDisp2Plot = new MouseMonitorPlot();
        xDisp2Plot->init(Qt::red, "X Displacement", "step", "cm");
        dispBox2->layout()->addWidget(xDisp2Plot);

        yDisp2Plot = new MouseMonitorPlot();
        yDisp2Plot->init(Qt::red, "Y Displacement", "step", "cm");
        dispBox2->layout()->addWidget(yDisp2Plot);

        xSpeed1Plot = new MouseMonitorPlot();
        xSpeed1Plot->init(Qt::blue, "X Speed", "step", "cm/s");
        speedBox1->layout()->addWidget(xSpeed1Plot);

        ySpeed1Plot = new MouseMonitorPlot();
        ySpeed1Plot->init(Qt::blue, "Y Speed", "step", "cm/s");
        speedBox1->layout()->addWidget(ySpeed1Plot);

        xSpeed2Plot = new MouseMonitorPlot();
        xSpeed2Plot->init(Qt::red, "X Speed", "step", "cm/s");
        speedBox2->layout()->addWidget(xSpeed2Plot);

        ySpeed2Plot = new MouseMonitorPlot();
        ySpeed2Plot->init(Qt::red, "Y Speed", "step", "cm/s");
        speedBox2->layout()->addWidget(ySpeed2Plot);
    }

    void MouseMonitorWindow::initDetail()
    {
        QStringList headerLabels;
        headerLabels << "X Displacement" << "Y Displacement"
                     << "X Speed" << "Y Speed";

        detail1Table->setColumnCount(4);

        detail1Table->setHorizontalHeaderLabels(headerLabels);
        detail1Table->setEditTriggers(QAbstractItemView::NoEditTriggers);
        detail1Table->horizontalHeader()->setResizeMode(QHeaderView::Stretch);

        detail2Table->setColumnCount(4);

        detail2Table->setHorizontalHeaderLabels(headerLabels);
        detail2Table->setEditTriggers(QAbstractItemView::NoEditTriggers);
        detail2Table->horizontalHeader()->setResizeMode(QHeaderView::Stretch);

        detail1Plot = new MouseMonitorPlot();
        detail1Plot->init(Qt::red, "X Displacement", "step", "cm");
        detail1Box->layout()->addWidget(detail1Plot);

        detail2Plot = new MouseMonitorPlot();
        detail2Plot->init(Qt::blue, "X Displacement", "step", "cm");
        detail2Box->layout()->addWidget(detail2Plot);
    }

    void MouseMonitorWindow::timerTimeout()
    {
        processMouseData(monitorNode.getMouseData(SENSOR1));
        processMouseData(monitorNode.getMouseData(SENSOR2));
    }

    /**
     * This function is called to update all display widgets with the new
     * sensor data.
     */
    void MouseMonitorWindow::processMouseData(const MouseData data)
    {
        updateData(data);
        pathWidget->updateWidget(data);
        updateDirectionWidgets(data);
        updatePlot(data);
        updateAbsValue(data);
    }

    void MouseMonitorWindow::updateAbsValue(MouseData data)
    {
        QString txt;
        double value;

        if (data.id == SENSOR1) {
            txt = x_disp1_abs->text();
            value = txt.toDouble();
            value += data.x_disp;
            x_disp1_abs->setText(QString("%1").arg(value, 0, 'f', 8));

            txt = y_disp1_abs->text();
            value = txt.toDouble();
            value += data.y_disp;
            y_disp1_abs->setText(QString("%1").arg(value, 0, 'f', 8));
        } else if (data.id == SENSOR2) {
            txt = x_disp2_abs->text();
            value = txt.toDouble();
            value += data.x_disp;
            x_disp2_abs->setText(QString("%1").arg(value, 0, 'f', 8));

            txt = y_disp2_abs->text();
            value = txt.toDouble();
            value += data.y_disp;
            y_disp2_abs->setText(QString("%1").arg(value, 0, 'f', 8));
        }
    }

    void MouseMonitorWindow::updatePlot(MouseData data)
    {
        if (data.id == SENSOR1) {
            xDisp1Plot->updatePlot(data.x_disp);
            yDisp1Plot->updatePlot(data.y_disp);
            xSpeed1Plot->updatePlot(data.x_speed);
            ySpeed1Plot->updatePlot(data.y_speed);
        } else if (data.id == SENSOR2) {
            xDisp2Plot->updatePlot(data.x_disp);
            yDisp2Plot->updatePlot(data.y_disp);
            xSpeed2Plot->updatePlot(data.x_speed);
            ySpeed2Plot->updatePlot(data.y_speed);
        }
    }

    void MouseMonitorWindow::updateDirectionWidgets(MouseData data)
    {
        if (data.id == SENSOR1) {
            directionWidget1->updateWidget(data);
        } else if (data.id == SENSOR2) {
            directionWidget2->updateWidget(data);
        }
    }

    void MouseMonitorWindow::updateData(MouseData data)
    {
        if (data.id == SENSOR1) {
            name1->setText(QString::fromStdString(data.id));
            x_disp1->setText(QString("%1").arg(data.x_disp, 0, 'f', 8));
            y_disp1->setText(QString("%1").arg(data.y_disp, 0, 'f', 8));
            x_speed1->setText(QString("%1").arg(data.x_speed, 0, 'f', 8));
            y_speed1->setText(QString("%1").arg(data.y_speed, 0, 'f', 8));
        } else if (data.id == SENSOR2) {
            name2->setText(QString::fromStdString(data.id));
            x_disp2->setText(QString("%1").arg(data.x_disp, 0, 'f', 8));
            y_disp2->setText(QString("%1").arg(data.y_disp, 0, 'f', 8));
            x_speed2->setText(QString("%1").arg(data.x_speed, 0, 'f', 8));
            y_speed2->setText(QString("%1").arg(data.y_speed, 0, 'f', 8));
        }
    }

    /**
     * This function is used to update the table with the sensors settings.
     */
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

    MouseMonitorNode& MouseMonitorWindow::getMonitorNode()
    {
        return monitorNode;
    }

    // The event handlers for the widgets

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
        int res = resolution1Edit->text().toInt();
        monitorNode.sendResolution(SENSOR1, res);
    }

    void MouseMonitorWindow::resolution2BtnClicked()
    {
        int res = resolution2Edit->text().toInt();
        monitorNode.sendResolution(SENSOR2, res);
    }


    void MouseMonitorWindow::getSensorSettingsBtnClicked()
    {
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

    void MouseMonitorWindow::trackPathResetBtnClicked()
    {
        x_disp1_abs->setText("0");
        y_disp1_abs->setText("0");
        x_disp2_abs->setText("0");
        y_disp2_abs->setText("0");

        pathWidget->reset();
    }

    void MouseMonitorWindow::detailDebuggingEnable(const int status)
    {
        if (status == 2) {
            timer->stop();
            detailDebugginFrame->setEnabled(true);
        } else {
            timer->start();
            detailDebugginFrame->setEnabled(false);
        }
    }

    void MouseMonitorWindow::getData1BtnClicked()
    {
        MouseData data = monitorNode.getMouseData(SENSOR1);

        QString xd = QString::number(data.x_disp);
        QString yd = QString::number(data.y_disp);
        QString txt;
        txt = xd + ", " + yd;

        detail1Edit->setText(txt);

        detail1Table->insertRow(0);
        detail1Table->setItem(0, 0, new QTableWidgetItem(QString("%1").arg(data.x_disp, 0, 'f')));
        detail1Table->setItem(0, 1, new QTableWidgetItem(QString("%1").arg(data.y_disp, 0, 'f')));
        detail1Table->setItem(0, 2, new QTableWidgetItem(QString("%1").arg(data.x_disp, 0, 'f')));
        detail1Table->setItem(0, 3, new QTableWidgetItem(QString("%1").arg(data.y_speed, 0, 'f')));

        detail1Plot->updatePlot(data.y_disp);
    }

    void MouseMonitorWindow::getData2BtnClicked()
    {
        MouseData data = monitorNode.getMouseData(SENSOR2);

        QString xd = QString::number(data.x_disp);
        QString yd = QString::number(data.y_disp);
        QString txt;
        txt = xd + ", " + yd;

        detail2Edit->setText(txt);

        detail2Table->insertRow(0);
        detail2Table->setItem(0, 0, new QTableWidgetItem(QString("%1").arg(data.x_disp, 0, 'f')));
        detail2Table->setItem(0, 1, new QTableWidgetItem(QString("%1").arg(data.y_disp, 0, 'f')));
        detail2Table->setItem(0, 2, new QTableWidgetItem(QString("%1").arg(data.x_disp, 0, 'f')));
        detail2Table->setItem(0, 3, new QTableWidgetItem(QString("%1").arg(data.y_speed, 0, 'f')));

        detail2Plot->updatePlot(data.y_disp);
    }

    QString MouseMonitorWindow::uintToQString(uint data)
    {
        return QString("%1").arg(data, 4, 16, QChar('0'));
    }

}

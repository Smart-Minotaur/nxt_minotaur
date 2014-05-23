#include "nxt_qt/MouseMonitorWindow.hpp"
#include <QPainter>
#include <QWidget>
#include <QMouseEvent>
#include <QToolTip>
#include <string>
#include "ros/ros.h"

// Direction grid params
#define GRID_SCALE 20

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

        connect(&monitorNode, SIGNAL(measuredMouseData(const MouseData)),
                this, SLOT(processMouseData(const MouseData)));

        connect(&monitorNode, SIGNAL(measuredMouseSettings(const std::string,
                                     const pln_minotaur::PLN2033_Settings)),
                this, SLOT(processMouseSettings(const std::string,
                                                const pln_minotaur::PLN2033_Settings)));

        connect(zoomSlider, SIGNAL(valueChanged(int)),
                pathWidget, SLOT(zoomValueChanged(const int)));

        connect(trackSensor1, SIGNAL(stateChanged(int)),
                pathWidget, SLOT(sensor1Enable(const int)));

        connect(trackSensor2, SIGNAL(stateChanged(int)),
                pathWidget, SLOT(sensor2Enable(const int)));
    }

    MouseMonitorWindow::~MouseMonitorWindow()
    {
        delete directionWidget1;
        delete directionWidget2;
        delete pathWidget;
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
        // TODO: Add table initialization
        sensorSettingsTable->setColumnCount(14);
        sensorSettingsTable->setRowCount(2);

        QStringList headerLabels;
        headerLabels << "Status Register" << "Delta X Disp" << "Delta Y Disp"
	<< "Command High" << "Command Low" << "Memory pointer" << "Memory Data"
	<< "Mode Control" << "Power Control" << "Mode Status" << "System Control"
	<< "Misc" << "Interrupt Output" << "Zu Viel";

        sensorSettingsTable->setHorizontalHeaderLabels(headerLabels);
	
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

    MouseMonitorNode& MouseMonitorWindow::getMonitorNode()
    {
        return monitorNode;
    }

    void MouseMonitorWindow::processMouseData(const MouseData data)
    {
        if (data.id == SENSOR1) {
            name1->setText(QString::fromStdString(data.id));
            x_disp1->setText(QString("%1").arg(data.x_disp, 0, 'f', 2));
            y_disp1->setText(QString("%1").arg(data.y_disp, 0, 'f', 2));
            x_speed1->setText(QString("%1").arg(data.x_speed, 0, 'f', 2));
            y_speed1->setText(QString("%1").arg(data.y_speed, 0, 'f', 2));

            directionWidget1->updateWidget(data);
            pathWidget->updateWidget(data);
            updatePlotSensor1(data);
        } else if (data.id == SENSOR2) {
            name2->setText(QString::fromStdString(data.id));
            x_disp2->setText(QString("%1").arg(data.x_disp, 0, 'f', 2));
            y_disp2->setText(QString("%1").arg(data.y_disp, 0, 'f', 2));
            x_speed2->setText(QString("%1").arg(data.x_speed, 0, 'f', 2));
            y_speed2->setText(QString("%1").arg(data.y_speed, 0, 'f', 2));

            directionWidget2->updateWidget(data);
            pathWidget->updateWidget(data);
            updatePlotSensor2(data);
        }
    }

    void MouseMonitorWindow::processMouseSettings(
        const std::string id,
        const pln_minotaur::PLN2033_Settings settings)
    {
        // TODO: Insert settings values into the table
        if (id == SENSOR1)
	{
	  sensorSettingsTable->setItem(0,0, new QTableWidgetItem(settings.status_register));
	  sensorSettingsTable->setItem(0, 1, new QTableWidgetItem(settings.delta_x_disp_register));
	  sensorSettingsTable->setItem(0, 2, new QTableWidgetItem(settings.delta_y_disp_register));
	  sensorSettingsTable->setItem(0, 3, new QTableWidgetItem(settings.command_high_register));
	  sensorSettingsTable->setItem(0, 4, new QTableWidgetItem(settings.command_low_register));
	  sensorSettingsTable->setItem(0, 5, new QTableWidgetItem(settings.memory_pointer_register));
	  sensorSettingsTable->setItem(0, 6, new QTableWidgetItem(settings.memory_data_register));
	  sensorSettingsTable->setItem(0, 7, new QTableWidgetItem(settings.mode_control_register));
	  sensorSettingsTable->setItem(0, 8, new QTableWidgetItem(settings.power_control_register));
	  sensorSettingsTable->setItem(0, 9, new QTableWidgetItem(settings.mode_status_register));
	  sensorSettingsTable->setItem(0, 10, new QTableWidgetItem(settings.system_control_register));
	  sensorSettingsTable->setItem(0, 11, new QTableWidgetItem(settings.miscellaneous_register));
	  sensorSettingsTable->setItem(0, 12, new QTableWidgetItem(settings.interrupt_output_register));
	  sensorSettingsTable->setItem(0, 13, new QTableWidgetItem(settings.status_register));
	  }
	  else
	  {
	    sensorSettingsTable->setItem(1,0, new QTableWidgetItem(settings.status_register));
	    sensorSettingsTable->setItem(1, 1, new QTableWidgetItem(settings.delta_x_disp_register));
	    sensorSettingsTable->setItem(1, 2, new QTableWidgetItem(settings.delta_y_disp_register));
	    sensorSettingsTable->setItem(1, 3, new QTableWidgetItem(settings.command_high_register));
	    sensorSettingsTable->setItem(1, 4, new QTableWidgetItem(settings.command_low_register));
	    sensorSettingsTable->setItem(1, 5, new QTableWidgetItem(settings.memory_pointer_register));
	    sensorSettingsTable->setItem(1, 6, new QTableWidgetItem(settings.memory_data_register));
	    sensorSettingsTable->setItem(1, 7, new QTableWidgetItem(settings.mode_control_register));
	    sensorSettingsTable->setItem(1, 8, new QTableWidgetItem(settings.power_control_register));
	    sensorSettingsTable->setItem(1, 9, new QTableWidgetItem(settings.mode_status_register));
	    sensorSettingsTable->setItem(1, 10, new QTableWidgetItem(settings.system_control_register));
	    sensorSettingsTable->setItem(1, 11, new QTableWidgetItem(settings.miscellaneous_register));
	    sensorSettingsTable->setItem(1, 12, new QTableWidgetItem(settings.interrupt_output_register));
	    sensorSettingsTable->setItem(1, 13, new QTableWidgetItem(settings.status_register));
	  }
	
    
    }

    // Widgets ========================================================

    void DirectionWidget::paintEvent(QPaintEvent *event)
    {
        QPainter painter(this);

        double gridWidth = this->width();
        double gridHeight = this->height();

        int xSteps = (int) (gridWidth / GRID_SCALE);
        int ySteps = (int) (gridHeight / GRID_SCALE);

        painter.setPen(QPen(Qt::gray, 1));

        for (int i = 0; i <= xSteps; ++i) {
            painter.drawLine(i * GRID_SCALE, 0,
                             i * GRID_SCALE, ySteps * GRID_SCALE);
        }

        for (int i = 0; i <= ySteps; ++i) {
            painter.drawLine(0, i * GRID_SCALE,
                             xSteps * GRID_SCALE, i * GRID_SCALE);
        }
    }

    void DirectionWidget::updateWidget(MouseData data)
    {
        // TODO: Update the data

        update();
    }

    void TrackPathWidget::init()
    {
        // TODO
        startx = geometry().width() / 2.0;
        starty = 20;

        sensor1_path.moveTo(QPointF(startx, starty));
        sensor2_path.moveTo(QPointF(startx, starty));

        translatex = 0.0;
        translatey = 0.0;
        lastMousePos.setX(0.0);
        lastMousePos.setY(0.0);

        setMouseTracking(true);

        zoom = 1;
        sensor1_enable = true;
        sensor2_enable = true;
    }

    void TrackPathWidget::paintEvent(QPaintEvent *event)
    {
        QPainter painter(this);

        // Draw grid
        double gridWidth = this->width();
        double gridHeight = this->height();

        int xSteps = (int) (gridWidth / GRID_SCALE);
        int ySteps = (int) (gridHeight / GRID_SCALE);

        painter.setPen(QPen(Qt::gray, 1));

        for (int i = 0; i <= xSteps; ++i) {
            painter.drawLine(i * GRID_SCALE, 0,
                             i * GRID_SCALE, ySteps * GRID_SCALE);
        }

        for (int i = 0; i <= ySteps; ++i) {
            painter.drawLine(0, i * GRID_SCALE,
                             xSteps * GRID_SCALE, i * GRID_SCALE);
        }

        painter.setRenderHint(QPainter::Antialiasing);
        painter.scale((qreal) zoom, (qreal) zoom);
        painter.translate(QPointF(translatex, translatey));

        // Draw path
        if (sensor1_enable) {
            painter.setPen(Qt::blue);
            painter.drawPath(sensor1_path);
        }

        if (sensor2_enable) {
            painter.setPen(Qt::red);
            painter.drawPath(sensor2_path);
        }
    }

    void TrackPathWidget::updateWidget(MouseData data)
    {
        if (data.id == "/dev/spidev1.0")
            sensor1_path.lineTo(sensor1_path.currentPosition() + QPointF(data.x_disp, data.y_disp));
        else if (data.id == "/dev/spidev1.1")
            sensor2_path.lineTo(sensor2_path.currentPosition() + QPointF(data.x_disp, data.y_disp));

        update();
    }

    void TrackPathWidget::zoomValueChanged(const int value)
    {
        zoom = value;
        update();
    }

    void TrackPathWidget::sensor1Enable(const int status)
    {
        if (status == 2)
            this->sensor1_enable = true;
        else
            this->sensor1_enable = false;

        update();
    }

    void TrackPathWidget::sensor2Enable(const int status)
    {
        if (status == 2)
            this->sensor2_enable = true;
        else
            this->sensor2_enable = false;

        update();
    }

    void TrackPathWidget::mouseMoveEvent(QMouseEvent *event)
    {
        translatex = event->x() - lastMousePos.x();
        translatey = event->y() - lastMousePos.y();

        lastMousePos = event->posF();

        QString str = "Last Mouse Pos";
        QToolTip::showText(this->mapToGlobal(lastMousePos.toPoint()), str, this);
    }

    void TrackPathWidget::mouseReleaseEvent(QMouseEvent *event)
    {
        lastMousePos.setX(0.0);
        lastMousePos.setY(0.0);
    }

}

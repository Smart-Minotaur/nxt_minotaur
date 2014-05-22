#include "nxt_qt/MouseMonitorWindow.hpp"
#include <QPainter>
#include <QWidget>
#include <string>
#include "ros/ros.h"

// grid params
#define AMPLIFY 1000
#define SCALE 20

// Plots
#define MAX_PLOT_DISP 2
#define PLOT_DISP_STEP MAX_PLOT_DISP / 10
#define MAX_PLOT_SPEED 50
#define PLOT_SPEED_STEP MAX_PLOT_SPEED / 10

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
        headerLabels << "Status Register";
        sensorSettingsTable->setHorizontalHeaderLabels(headerLabels);
    }

    void MouseMonitorWindow::initPlots()
    {
        memset(xDisp1, 0, SAMPLE_RANGE * sizeof(double));
        memset(yDisp1, 0, SAMPLE_RANGE * sizeof(double));
        memset(xDisp2, 0, SAMPLE_RANGE * sizeof(double));
        memset(yDisp2, 0, SAMPLE_RANGE * sizeof(double));

        for(int i = 0; i < SAMPLE_RANGE; ++i) {
            xSamplesSensor1[i] = i;
            xSamplesSensor2[i] = i;
        }

        sampleCountSensor1 = 0;
        sampleCountSensor2 = 0;

        xDisp1Curve.setPen(QColor(Qt::blue));
        xDisp1Curve.setSamples(xSamplesSensor1, xDisp1, SAMPLE_RANGE);
        xDisp1Curve.attach(x_disp1_viz);

        x_disp1_viz->setTitle("X Displacement");
        x_disp1_viz->setAxisScale(QwtPlot::xBottom, 0, SAMPLE_RANGE, 50);
        x_disp1_viz->setAxisScale(QwtPlot::yLeft, -MAX_PLOT_DISP, MAX_PLOT_DISP, PLOT_DISP_STEP);
        x_disp1_viz->setAxisTitle(QwtPlot::xBottom, "step");
        x_disp1_viz->setAxisTitle(QwtPlot::yLeft, "cm");

        yDisp1Curve.setPen(QColor(Qt::blue));
        yDisp1Curve.setSamples(xSamplesSensor1, yDisp1, SAMPLE_RANGE);
        yDisp1Curve.attach(y_disp1_viz);

        y_disp1_viz->setTitle("Y Displacement");
        y_disp1_viz->setAxisScale(QwtPlot::xBottom, 0, SAMPLE_RANGE, 50);
        y_disp1_viz->setAxisScale(QwtPlot::yLeft, -MAX_PLOT_DISP, MAX_PLOT_DISP, PLOT_DISP_STEP);
        y_disp1_viz->setAxisTitle(QwtPlot::xBottom, "step");
        y_disp1_viz->setAxisTitle(QwtPlot::yLeft, "cm");

        xDisp2Curve.setPen(QColor(Qt::red));
        xDisp2Curve.setSamples(xSamplesSensor2, xDisp2, SAMPLE_RANGE);
        xDisp2Curve.attach(x_disp2_viz);

        x_disp2_viz->setTitle("X Displacement");
        x_disp2_viz->setAxisScale(QwtPlot::xBottom, 0, SAMPLE_RANGE, 50);
        x_disp2_viz->setAxisScale(QwtPlot::yLeft, -MAX_PLOT_DISP, MAX_PLOT_DISP, PLOT_DISP_STEP);
        x_disp2_viz->setAxisTitle(QwtPlot::xBottom, "step");
        x_disp2_viz->setAxisTitle(QwtPlot::yLeft, "cm");

        yDisp2Curve.setPen(QColor(Qt::red));
        yDisp2Curve.setSamples(xSamplesSensor2, yDisp2, SAMPLE_RANGE);
        yDisp2Curve.attach(y_disp2_viz);

        y_disp2_viz->setTitle("Y Displacement");
        y_disp2_viz->setAxisScale(QwtPlot::xBottom, 0, SAMPLE_RANGE, 50);
        y_disp2_viz->setAxisScale(QwtPlot::yLeft, -MAX_PLOT_DISP, MAX_PLOT_DISP, PLOT_DISP_STEP);
        y_disp2_viz->setAxisTitle(QwtPlot::xBottom, "step");
        y_disp2_viz->setAxisTitle(QwtPlot::yLeft, "cm");
    }

    void MouseMonitorWindow::updatePlotSensor1(MouseData data)
    {
        xDisp1[sampleCountSensor1] = data.x_disp;
        yDisp1[sampleCountSensor1] = data.y_disp;

        xDisp1Curve.setSamples(xSamplesSensor1, xDisp1, SAMPLE_RANGE);
        yDisp1Curve.setSamples(xSamplesSensor1, yDisp1, SAMPLE_RANGE);

        x_disp1_viz->replot();
        y_disp1_viz->replot();

        ++sampleCountSensor1;
        sampleCountSensor1 %= SAMPLE_RANGE;
    }

    void MouseMonitorWindow::updatePlotSensor2(MouseData data)
    {
        xDisp2[sampleCountSensor2] = data.x_disp;
        yDisp2[sampleCountSensor2] = data.y_disp;

        xDisp2Curve.setSamples(xSamplesSensor2, xDisp2, SAMPLE_RANGE);
        yDisp2Curve.setSamples(xSamplesSensor2, yDisp2, SAMPLE_RANGE);

        x_disp2_viz->replot();
        y_disp2_viz->replot();

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
    }

    // Widgets ========================================================

    void DirectionWidget::paintEvent(QPaintEvent *event)
    {
        QPainter painter(this);

        // Add the vertical lines first, paint them
        painter.setPen(QPen(Qt::black, 1));
        for (int x = 0; x <= this->width(); x += SCALE)
            painter.drawLine(x, 0, x, this->height());

        // Now add the horizontal lines, paint them
        for (int y = 0; y <= this->height(); y += SCALE)
            painter.drawLine(0, y, this->width(), y);
    }

    void DirectionWidget::updateWidget(MouseData data)
    {
        this->data = data;
        update();
    }

    void TrackPathWidget::paintEvent(QPaintEvent *event)
    {
        QPainter painter(this);
        painter.setRenderHint(QPainter::Antialiasing);

        if (data.id == "/dev/spidev1.0")
            painter.setPen(Qt::blue);
        else if (data.id == "/dev/spidev1.1")
            painter.setPen(Qt::red);

        path.lineTo(path.currentPosition() + QPointF(data.x_disp, data.y_disp));
        painter.drawPath(path);
    }

    void TrackPathWidget::updateWidget(MouseData data)
    {
        this->data = data;
        update();
    }

}

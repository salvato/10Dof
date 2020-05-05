#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "plot2d.h"
#include <QtWidgets>
#include <QDebug>
#include <QThread>
#include <GLwidget.h>
#include <QPushButton>

#include <cmath>

// Hardware Connections:
//
// DFRobot 10DOF :
//      SDA on BCM 2:    pin 3 in the 40 pins GPIO connector
//      SDL on BCM 3:    pin 5 in the 40 pins GPIO connector
//      Vcc on 5V Power: pin 4 in the 40 pins GPIO connector
//      GND on GND:      pin 6 in the 40 pins GPIO connector
//
// For Raspberry Pi GPIO pin numbering see
// https://pinout.xyz/
//
// +5V on pins 2 or 4 in the 40 pin GPIO connector.
// GND on pins 6, 9, 14, 20, 25, 30, 34 or 39
// in the 40 pin GPIO connector.

// PWM 0 is on BCM12: Pin 32 in the 40 pin GPIO connector.
// PWM 1 is on BCM13: Pin 33 in the 40 pin GPIO connector.

// #define PAN_PIN  14 // GPIO Numbers are Broadcom (BCM) numbers
// #define TILT_PIN 26 // GPIO Numbers are Broadcom (BCM) numbers

//==============================================================
// Informations for connecting servos:
//
// Samwa servo pinout:
//      1) PWM Signal
//      2) GND
//      3) +5V
//==============================================================

// I2C Addresses :
//      0x1E     HMC5883L_Address        (Magnetometer)
//      0x53     ADXL345_ADDR_ALT_LOW    (Accelerometer)
//      0x68     ITG3200_ADDR_AD0_LOW    (Gyroscope)
//      0x77     BMP085_ADDRESS          (Pressure and Temperature)


MainWindow::MainWindow()
    : pGLWidget(nullptr)
    , pPlotVal(nullptr)
    , bAccCalInProgress(false)
    , bGyroCalInProgress(false)
    , bMagCalInProgress(false)
{

    buttonAccCalibration  = new QPushButton("Acc. Cal.", this);
    buttonGyroCalibration = new QPushButton("Gyro Cal.", this);
    buttonMagCalibration  = new QPushButton("Mag. Cal.", this);

    connect(buttonAccCalibration, SIGNAL(clicked()),
            this, SLOT(onStartAccCalibration()));
    connect(buttonGyroCalibration, SIGNAL(clicked()),
            this, SLOT(onStartGyroCalibration()));
    connect(buttonMagCalibration, SIGNAL(clicked()),
            this, SLOT(onStartMagCalibration()));

    pGLWidget = new GLWidget(this);

    pPlotVal = new Plot2D(this, "Plot");

    pPlotVal->NewDataSet(1, 1, QColor(255, 0, 0), Plot2D::ipoint, "X");
    pPlotVal->NewDataSet(2, 1, QColor(0, 255, 0), Plot2D::ipoint, "Y");
    pPlotVal->NewDataSet(3, 1, QColor(0, 0, 255), Plot2D::ipoint, "Z");

    pPlotVal->SetLimits(-1.0, 1.0, -1.0, 1.0, true, true, false, false);

    pPlotVal->SetShowDataSet(1, true);
    pPlotVal->SetShowDataSet(2, true);
    pPlotVal->SetShowDataSet(3, true);

    initLayout();

    pAcc  = new ADXL345(); // init ADXL345
    pAcc->init(FIMU_ACC_ADDR);

    pGyro = new ITG3200(); // init ITG3200
    pGyro->init(FIMU_ITG3200_DEF_ADDR);
    QThread::msleep(1000);
    pGyro->zeroCalibrate(128, 5); // calibrate the ITG3200

    pMagn = new HMC5883L();// init HMC5883L
    // The magnitude of the Earth's magnetic field at its surface
    // ranges from 250 to 650 milli Gauss.
    int16_t error = pMagn->SetScale(1300); // Set the scale (in milli Gauss) of the compass.
    if(error != 0) {
        qDebug() << pMagn->GetErrorText(error);
        //exit(EXIT_FAILURE);
    }
    error = pMagn->SetMeasurementMode(Measurement_Continuous); // Set the measurement mode to Continuous
    if(error != 0)  {
        qDebug() << pMagn->GetErrorText(error);
        //exit(EXIT_FAILURE);
    }

    //bmp085Calibration(); // init barometric pressure sensor

    pMadgwick = new Madgwick();

    sampleFrequency = 200;
    pMadgwick->begin(sampleFrequency);

    // Consider to change to QBasicTimer that it's faster than QTimer
    loopTimer.setTimerType(Qt::PreciseTimer);
    connect(&loopTimer, SIGNAL(timeout()),
            this, SLOT(onLoopTimeElapsed()));

    while(!pAcc->getInterruptSource(7)) {}
    pAcc->get_Gxyz(&values[0]);
    while(!pGyro->isRawDataReadyOn()) {}
    pGyro->readGyro(&values[3]);
    while(!pMagn->isDataReady()) {}
    pMagn->ReadScaledAxis(&values[6]);

    for(int i=0; i< 10000; i++) {
        pMadgwick->update(values[3], values[4], values[5],
                values[0], values[1], values[2],
                values[6], values[7], values[8]);
    }
    pMadgwick->getRotation(&q0, &q1, &q2, &q3);
    pGLWidget->setRotation(q0, q1, q2, q3);
    lastUpdate = micros();
    now = micros();
    loopTimer.start(int32_t(1000.0/sampleFrequency+0.5));
}


MainWindow::~MainWindow() {
    if(pGLWidget) delete pGLWidget;
    pGLWidget = nullptr;
}


void
MainWindow::closeEvent(QCloseEvent *event) {
    Q_UNUSED(event)
    loopTimer.stop();
    if(pGLWidget) delete pGLWidget;
    pGLWidget = nullptr;
    if(pPlotVal) delete pPlotVal;
    pPlotVal = nullptr;
}



void
MainWindow::initLayout() {
    QHBoxLayout *firstButtonRow = new QHBoxLayout;
    firstButtonRow->addWidget(buttonAccCalibration);
    firstButtonRow->addWidget(buttonGyroCalibration);
    firstButtonRow->addWidget(buttonMagCalibration);

    QHBoxLayout *firstRow  = new QHBoxLayout;
    firstRow->addWidget(pGLWidget);
    firstRow->addWidget(pPlotVal);

    QVBoxLayout *mainLayout = new QVBoxLayout;
    mainLayout->addLayout(firstRow);
    mainLayout->addLayout(firstButtonRow);

    setLayout(mainLayout);
}


void
MainWindow::keyPressEvent(QKeyEvent *e) {
  if(e->key() == Qt::Key_Escape)
    close();
  else
    QWidget::keyPressEvent(e);
}


void
MainWindow::onStartAccCalibration() {
    if(bAccCalInProgress) {
        buttonAccCalibration->setText("Acc. Cal.");
        bAccCalInProgress = false;
        buttonGyroCalibration->setEnabled(true);
        buttonMagCalibration->setEnabled(true);
    }
    else {
        pPlotVal->ClearPlot();
        pPlotVal->NewDataSet(1, 1, QColor(255, 0, 0), Plot2D::ipoint, "X");
        pPlotVal->NewDataSet(2, 1, QColor(0, 255, 0), Plot2D::ipoint, "Y");
        pPlotVal->NewDataSet(3, 1, QColor(0, 0, 255), Plot2D::ipoint, "Z");
        pPlotVal->SetLimits(-1.0, 1.0, -1.0, 1.0, true, true, false, false);
        pPlotVal->SetShowDataSet(1, true);
        pPlotVal->SetShowDataSet(2, true);
        pPlotVal->SetShowDataSet(3, true);
        bGyroCalInProgress = false;
        bMagCalInProgress = false;
        bAccCalInProgress = true;
        buttonAccCalibration->setText("Stop Cal.");
        buttonGyroCalibration->setDisabled(true);
        buttonMagCalibration->setDisabled(true);
        avgX = avgY = avgZ = 0.0;
        nAvg = 10;
        nCurr = 0;
        t0 = micros();
    }
}


void
MainWindow::onStartGyroCalibration() {
    if(bGyroCalInProgress) {
        buttonGyroCalibration->setText("Gyro Cal.");
        bGyroCalInProgress = false;
        buttonAccCalibration->setEnabled(true);
        buttonMagCalibration->setEnabled(true);
    }
    else {
        pPlotVal->ClearPlot();
        pPlotVal->NewDataSet(1, 1, QColor(255, 0, 0), Plot2D::ipoint, "X");
        pPlotVal->NewDataSet(2, 1, QColor(0, 255, 0), Plot2D::ipoint, "Y");
        pPlotVal->NewDataSet(3, 1, QColor(0, 0, 255), Plot2D::ipoint, "Z");
        pPlotVal->SetLimits(-1.0, 1.0, -1.0, 1.0, true, true, false, false);
        pPlotVal->SetShowDataSet(1, true);
        pPlotVal->SetShowDataSet(2, true);
        pPlotVal->SetShowDataSet(3, true);
        bMagCalInProgress = false;
        bAccCalInProgress = false;
        bGyroCalInProgress = true;
        buttonGyroCalibration->setText("Stop Cal.");
        buttonAccCalibration->setDisabled(true);
        buttonMagCalibration->setDisabled(true);
        angleX = 0.0;
        angleY = 0.0;
        angleZ = 0.0;
        t0 = micros();
    }
}


void
MainWindow::onStartMagCalibration() {
    if(bMagCalInProgress) {
        bMagCalInProgress = false;
        buttonMagCalibration->setText("Mag. Cal.");
        buttonAccCalibration->setEnabled(true);
        buttonGyroCalibration->setEnabled(true);
    }
    else {
        pPlotVal->ClearPlot();
        pPlotVal->NewDataSet(1, 1, QColor(255, 0, 0), Plot2D::ipoint, "X");
        pPlotVal->NewDataSet(2, 1, QColor(0, 255, 0), Plot2D::ipoint, "Y");
        pPlotVal->NewDataSet(3, 1, QColor(0, 0, 255), Plot2D::ipoint, "Z");
        pPlotVal->SetLimits(-1.0, 1.0, -1.0, 1.0, true, true, false, false);
        pPlotVal->SetShowDataSet(1, true);
        pPlotVal->SetShowDataSet(2, true);
        pPlotVal->SetShowDataSet(3, true);
        buttonMagCalibration->setText("Stop Cal.");
        bAccCalInProgress = false;
        bGyroCalInProgress = false;
        bMagCalInProgress = true;
        buttonAccCalibration->setDisabled(true);
        buttonGyroCalibration->setDisabled(true);
        t0 = micros();
    }
}


void
MainWindow::onLoopTimeElapsed() {
    //
    //  !!! Warning !!!
    //
    // Reasonable convergence can be achieved in two or three iterations
    // meaning that we would like this sensor fusion filter to operate
    // a rate two or three times the output data rate of the sensor.
    //
    // Within the Madgwick algorithm deliver your values
    // in the expected format which is rad/s and m/s²

    if(pAcc->getInterruptSource(7)) {
        pAcc->get_Gxyz(&values[0]);
        if(bAccCalInProgress) {
            double x = (micros()-t0)/1000000.0;
            avgX += double(values[0]);
            avgY += double(values[1]);
            avgZ += double(values[2]);
            nCurr++;
            if(nCurr == nAvg) {
                avgX /= double(nAvg);
                avgY /= double(nAvg);
                avgZ /= double(nAvg);
                nCurr = 0;
                pPlotVal->NewPoint(1, x, avgX);
                pPlotVal->NewPoint(2, x, avgY);
                pPlotVal->NewPoint(3, x, avgZ);
            }
            avgX = avgY = avgZ = 0.0;
        }
    }

    if(pGyro->isRawDataReadyOn()) {
        pGyro->readGyro(&values[3]);
        if(bGyroCalInProgress) {
            double x = micros();
            double d = lastUpdate - x;
            x = (x-t0)/1000000.0;
            d /= 1000000.0;
            angleX += double(values[3]) * d;
            angleY += double(values[4]) * d;
            angleZ += double(values[5]) * d;
            pPlotVal->NewPoint(1, x, angleX);
            pPlotVal->NewPoint(2, x, angleY);
            pPlotVal->NewPoint(3, x, angleZ);
        }
    }

    if(pMagn->isDataReady()) {
        pMagn->ReadScaledAxis(&values[6]);
        if(bMagCalInProgress) {
            pPlotVal->NewPoint(1, double(values[6]), double(values[7]));
            pPlotVal->NewPoint(2, double(values[7]), double(values[8]));
            pPlotVal->NewPoint(3, double(values[8]), double(values[6]));
        }
    }
    now = micros();
    delta = float(now-lastUpdate)/1000000.f;
    pMadgwick->setInvFreq(delta);
    lastUpdate = now;
    pMadgwick->update(values[3], values[4], values[5],
            values[0], values[1], values[2],
            values[6], values[7], values[8]);

//    pMadgwick->updateIMU(values[3], values[4], values[5],
//                         values[0], values[1], values[2]);

    nUpdate++;
    nUpdate = nUpdate % 10;
    if(!nUpdate) {
        pMadgwick->getRotation(&q0, &q1, &q2, &q3);
        pGLWidget->setRotation(q0, q1, q2, q3);
        pGLWidget->update();
        pPlotVal->UpdatePlot();
    }
}
